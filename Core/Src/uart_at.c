#include "uart_at.h"
#include "at_config.h"
#include "config.h"
#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>

// The USART2 handle is defined as huart2 in main.c
extern UART_HandleTypeDef huart2;

// ==================== 静态变量 ====================

// RX环形缓冲区
static uint8_t s_rx_buf[UART_AT_RX_BUF_SIZE];
static volatile uint16_t s_rx_tail = 0;   // 读指针（任务消费）
static volatile uint16_t s_rx_write = 0;   // 写指针（ISR写入）

// 同步信号量（命令发送后等响应）
static osSemaphoreId_t s_cmd_sem = NULL;

// 命令结果（OK=1, ERROR=0, TIMEOUT=-1, NO_MATCH=2）
static volatile int s_cmd_result = 0;

// URC回调表
#define URC_MAX 8
typedef struct {
    char keyword[24];
    urc_callback_t cb;
    bool used;
} urc_entry_t;
static urc_entry_t s_urc_tbl[URC_MAX];

// UART模式（AT命令 vs XMODEM原始数据）
static volatile uart_at_mode_t s_uart_mode = UART_AT_MODE_AT;
static xmodem_byte_cb_t s_xmodem_byte_cb = NULL;

// ==================== 内部函数 ====================

// 启动一次IDLE中断接收，从rx_buf[s_rx_write]开始
static void arm_idle_rx(void) {
    uint16_t tail_copy, write_copy;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    tail_copy = s_rx_tail;
    write_copy = s_rx_write;
    __set_PRIMASK(primask);

    uint16_t used = (write_copy >= tail_copy)
                    ? (write_copy - tail_copy)
                    : (UART_AT_RX_BUF_SIZE - tail_copy + write_copy);

    if (used < UART_AT_RX_BUF_SIZE - 1) {
        uint16_t remaining = UART_AT_RX_BUF_SIZE - write_copy;
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, &s_rx_buf[write_copy], remaining);
    } else {
        // 缓冲区几乎满，记录溢出（可用回调通知，这里简单忽略）
    }
}

// UART RX事件/IDLE回调（由HAL_UART_IRQHandler调用）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance != USART2) return;

    if (Size > 0) {
        if (s_uart_mode == UART_AT_MODE_XMODEM && s_xmodem_byte_cb != NULL) {
            // XMODEM模式：直接回调每个字节，由XMODEM接收器处理
            // 此模式下不经过环形缓冲区，避免AT命令解析干扰
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            uint16_t write_copy = s_rx_write;
            s_rx_write = (s_rx_write + Size) % UART_AT_RX_BUF_SIZE;
            __set_PRIMASK(primask);

            for (uint16_t i = 0; i < Size; i++) {
                uint16_t idx = (write_copy + i) % UART_AT_RX_BUF_SIZE;
                s_xmodem_byte_cb(s_rx_buf[idx]);
            }
        } else {
            // AT命令模式：数据进入环形缓冲区，由AT解析器处理
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            s_rx_write = (s_rx_write + Size) % UART_AT_RX_BUF_SIZE;
            __set_PRIMASK(primask);
        }
    }

    // 重新启动下一轮IDLE接收
    arm_idle_rx();
}

// 解析环形缓冲区中的行，处理OK/ERROR/URC
static void parse_rx_lines(void) {
    char line[256];
    uint16_t i = 0;
    bool in_line = false;

    while (s_rx_tail != s_rx_write) {
        uint8_t ch = s_rx_buf[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) % UART_AT_RX_BUF_SIZE;

        if (ch == '\r' || ch == '\n') {
            if (in_line && i > 0) {
                line[i] = '\0';
                in_line = false;

                // 检查OK
                if (s_cmd_result == 0 &&
                    (strstr(line, "OK") != NULL || strcmp(line, "OK") == 0)) {
                    s_cmd_result = 1;
                    if (s_cmd_sem != NULL) osSemaphoreRelease(s_cmd_sem);
                }
                // 检查ERROR
                else if (s_cmd_result == 0 &&
                         (strstr(line, "ERROR") != NULL ||
                          strstr(line, "+CME ERROR") != NULL ||
                          strstr(line, "+CMS ERROR") != NULL)) {
                    s_cmd_result = 0;
                    if (s_cmd_sem != NULL) osSemaphoreRelease(s_cmd_sem);
                }
                // 检查URC（无论是否在等命令响应）
                {
                    for (int j = 0; j < URC_MAX; j++) {
                        if (s_urc_tbl[j].used && strstr(line, s_urc_tbl[j].keyword) != NULL) {
                            if (s_urc_tbl[j].cb) s_urc_tbl[j].cb(line);
                            break;
                        }
                    }
                }
                i = 0;
            }
        } else if (i < sizeof(line) - 1) {
            line[i++] = (char)ch;
            in_line = true;
        }
    }
}

// ==================== 公开API ====================

void uart_at_init(void) {
    // 初始化环形缓冲区
    s_rx_tail = 0;
    s_rx_write = 0;
    s_cmd_result = 0;

    // 创建二值信号量
    osSemaphoreAttr_t sem_attr = { .name = "at_cmd" };
    s_cmd_sem = osSemaphoreNew(1, 0, &sem_attr);

    // 初始化URC表
    memset(s_urc_tbl, 0, sizeof(s_urc_tbl));

    // 启动IDLE中断接收
    arm_idle_rx();
}

int at_send_command(const char *cmd, const char *expected_ok, uint32_t timeout_ms,
                    char *response, int resp_len) {
    (void)expected_ok; // 当前实现只检测OK，不检测自定义expected_ok
    if (cmd == NULL) return -2;

    at_flush_rx();

    char cmd_buf[320];
    snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);
    HAL_UART_Transmit(&huart2, (uint8_t *)cmd_buf, strlen(cmd_buf), 100);

    // 在等待前先解析已到达的数据（包含部分响应或URC）
    parse_rx_lines();

    s_cmd_result = 0;
    if (osSemaphoreAcquire(s_cmd_sem, timeout_ms) != osOK) {
        return -1; // timeout
    }

    // 复制响应数据（消耗环形缓冲区内容）
    if (response != NULL && resp_len > 0) {
        uint16_t copied = 0;
        while (s_rx_tail != s_rx_write && copied < (uint16_t)(resp_len - 1)) {
            response[copied++] = s_rx_buf[s_rx_tail];
            s_rx_tail = (s_rx_tail + 1) % UART_AT_RX_BUF_SIZE;
        }
        response[copied] = '\0';
    }

    return (s_cmd_result == 1) ? 0 : -2;
}

void at_send_raw(const uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0) return;
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100);
}

void at_register_urc(const char *keyword, urc_callback_t callback) {
    if (keyword == NULL || callback == NULL) return;
    for (int i = 0; i < URC_MAX; i++) {
        if (!s_urc_tbl[i].used) {
            strncpy(s_urc_tbl[i].keyword, keyword, sizeof(s_urc_tbl[i].keyword) - 1);
            s_urc_tbl[i].keyword[sizeof(s_urc_tbl[i].keyword) - 1] = '\0';
            s_urc_tbl[i].cb = callback;
            s_urc_tbl[i].used = true;
            break;
        }
    }
}

void at_flush_rx(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_rx_tail = 0;
    s_rx_write = 0;
    __set_PRIMASK(primask);
}

// ==================== UART模式切换 ====================

void uart_at_set_mode(uart_at_mode_t mode, xmodem_byte_cb_t xmodem_cb) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_uart_mode = mode;
    s_xmodem_byte_cb = xmodem_cb;
    // 切换到XMODEM模式时，不清空缓冲区（可能需要保留尾部残留数据）
    // 切换回AT模式时，清空缓冲区以避免残留XMODEM数据干扰AT解析
    if (mode == UART_AT_MODE_AT) {
        s_rx_tail = 0;
        s_rx_write = 0;
    }
    __set_PRIMASK(primask);
}

uart_at_mode_t uart_at_get_mode(void) {
    return s_uart_mode;
}
