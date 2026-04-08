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
/*---------------------------------------------------------------------------
 Name        : arm_idle_rx
 Input       : None
 Output      : None
 Description :
 启动/续启一次 USART2 的“接收到 IDLE 事件即回调”的中断接收（HAL ReceiveToIdle_IT），
 将接收 DMA/中断写入的位置指向环形缓冲区的当前写指针 `s_rx_write`。

 设计目的：
 - 利用 IDLE 中断把串口连续数据按“到达一批”通知上层（`HAL_UARTEx_RxEventCallback`），
   适合 AT 交互这种不定长、以行结束符为边界的数据流。

 关键变量约定：
 - s_rx_buf[]：环形缓冲区存储区
 - s_rx_write：写指针（由 ISR/HAL 回调推进）
 - s_rx_tail：读指针（由任务侧解析推进）

 实现要点：
 - 先在临界区内复制 tail/write，避免与 ISR 并发导致 used 计算不一致；
 - 计算当前已用字节 used，若缓冲区未满则启动下一次接收：
   - remaining = UART_AT_RX_BUF_SIZE - s_rx_write
   - 从 s_rx_buf[s_rx_write] 开始接收 remaining 字节

 注意事项：
 - 当前实现只启动“从写指针到缓冲区末尾”的一段接收，未直接处理写指针回绕后的连续接收；
   回绕由下一次回调推进 s_rx_write 后，再次调用 arm_idle_rx 重新 arm 来覆盖。
 - 当 used 接近满（>= BUF_SIZE-1）时目前仅注释提示，未上报溢出/丢弃策略；
   若 AT/URC 流量大，建议增加溢出计数与保护策略（如丢弃最旧数据或暂停接收）。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : HAL_UARTEx_RxEventCallback
 Input       : UART_HandleTypeDef *huart, uint16_t Size
 Output      : None
 Description :
 HAL 的“接收到数据/IDLE”事件回调入口（USART2），负责把新到达的数据并入 AT 接收框架：
 - AT 模式：仅推进写指针，让任务侧 `parse_rx_lines()` 解析整行并识别 OK/ERROR/URC；
 - XMODEM 模式：逐字节回调给 XMODEM 接收器（避免 AT 行解析干扰），并推进写指针。

 输入参数：
 - huart：HAL UART 句柄指针；本工程只处理 USART2（AT 口）
 - Size：本次回调新增接收的字节数（HAL ReceiveToIdle_IT 提供）

 处理逻辑（按当前实现）：
 - 若 huart 不是 USART2，直接返回；
 - Size>0 时：
   - 若处于 UART_AT_MODE_XMODEM 且已注册 `s_xmodem_byte_cb`：
     - 在临界区内记录 write_copy，并推进 s_rx_write（用于定位数据在 s_rx_buf 中的位置）
     - 逐字节调用 s_xmodem_byte_cb(s_rx_buf[idx]) 交给 XMODEM 状态机处理
   - 否则（AT 模式）：
     - 在临界区内仅推进 s_rx_write
 - 最后调用 `arm_idle_rx()` 重新启动下一轮接收

 并发/时序注意：
 - 该回调处于中断上下文（由 `HAL_UART_IRQHandler` 触发），必须保持短小；
 - XMODEM 模式逐字节回调会增加 ISR 时长，需确保回调函数为纯状态机、无阻塞、无动态分配；
 - s_rx_write/s_rx_tail 使用 volatile 并在临界区内更新，避免并发读写撕裂。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : parse_rx_lines
 Input       : None
 Output      : None
 Description :
 从环形缓冲区中按行提取文本（以 '\\r' 或 '\\n' 作为行结束符），并对每一行执行：
 - 命令响应判定：识别 "OK"、"ERROR"/"+CME ERROR"/"+CMS ERROR"
 - URC 分发：按关键字表 `s_urc_tbl[]` 匹配并调用对应回调

 行缓冲与解析策略：
 - 使用本地 `line[256]` 累积字符；
 - 遇到 '\\r' 或 '\\n'：
   - 若此前正在累积且长度>0，则认为完成一行，line 终止并开始处理；
 - 非换行字符且未超过 line 缓冲上限则继续累积。

 命令同步语义：
 - `s_cmd_result` 用于表示“当前是否在等待一个命令的最终结果”：
   - 当 s_cmd_result==0 时，首次识别到 OK -> s_cmd_result=1 并释放 `s_cmd_sem`
   - 当 s_cmd_result==0 时，首次识别到 ERROR -> s_cmd_result=0 并释放 `s_cmd_sem`
 - 释放信号量后，`at_send_command()` 会从任务上下文继续执行并复制响应。

 URC 语义：
 - URC（异步上报）不依赖是否在等命令响应：每行都会尝试关键字匹配；
 - 匹配策略为 strstr，命中第一个 used 项后调用其 cb(line)。

 注意事项/局限：
 - line 缓冲固定 256 字节，超长行会被截断（后续字符会被忽略直到换行）；
 - OK 匹配使用 strstr(line,"OK")，存在“包含 OK 子串的非 OK 行”误判风险；
 - 当前函数在任务上下文被调用（由 at_send_command 在发送后调用一次），
   并不会自动周期性解析；若 URC 在空闲期到达，需要由其他机制触发解析才能及时分发。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : uart_at_init
 Input       : None
 Output      : None
 Description :
 初始化 AT 串口接收与解析框架（USART2）。

 初始化内容：
 - 清空环形缓冲区读写指针 `s_rx_tail/s_rx_write`；
 - 复位命令结果 `s_cmd_result`；
 - 创建二值信号量 `s_cmd_sem`：
   - 用于 `at_send_command()` 在发送 AT 命令后等待 OK/ERROR 结果；
 - 清空 URC 回调表 `s_urc_tbl[]`；
 - 调用 `arm_idle_rx()` 启动第一轮 ReceiveToIdle 中断接收。

 调用时机：
 - 系统启动后、开始执行任何 AT 指令前调用一次（例如在 app_main_init() 中）。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : at_send_command
 Input       : const char *cmd, const char *expected_ok, uint32_t timeout_ms,
               char *response, int resp_len
 Output      : int
 Description :
 发送一条 AT 命令并阻塞等待结果（OK/ERROR 或超时），可选复制接收到的原始响应到 response。

 输入参数：
 - cmd：AT 命令字符串（不含结尾 CRLF）；不能为空
 - expected_ok：期望匹配的 OK 字符串（当前实现未使用，仅保留接口兼容）
 - timeout_ms：等待超时（毫秒）
 - response：输出缓冲区（可为 NULL）；用于获取原始接收内容（从环形缓冲区复制）
 - resp_len：response 缓冲区长度

 执行流程（按当前实现）：
 - `at_flush_rx()` 清空环形缓冲区，避免历史数据干扰本次命令判定；
 - 发送 cmd + "\\r\\n" 到 USART2（阻塞发送，超时 100ms）；
 - 调用 `parse_rx_lines()` 先解析已到达的数据（包含可能提前到达的 OK/URC）；
 - 将 `s_cmd_result` 置 0，随后等待 `s_cmd_sem`：
   - `parse_rx_lines()` 在识别到 OK/ERROR 时释放信号量；
 - 等到信号量后，从环形缓冲区复制剩余数据到 response（并推进 s_rx_tail）。

 返回值约定（当前实现）：
 - 返回 0：收到 OK（s_cmd_result==1）
 - 返回 -1：等待超时（osSemaphoreAcquire 超时）
 - 返回 -2：参数错误（cmd==NULL）或收到 ERROR（s_cmd_result!=1）

 重要说明/局限：
 - `expected_ok` 当前被忽略：仅通过 parse_rx_lines() 识别 "OK"/"ERROR"；
 - `at_flush_rx()` 会丢弃所有未处理的 URC/响应，若系统依赖 URC（如 MQTTURC），
   在频繁发命令时可能产生丢失风险，需要更精细的缓冲与匹配策略。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : at_send_raw
 Input       : const uint8_t *data, uint16_t len
 Output      : None
 Description :
 在 USART2 上发送原始字节流（不添加 CRLF，不进行 AT 行级解析），用于证书写入、
 XMODEM 数据、或模组要求的二进制透传场景。

 输入参数：
 - data：待发送数据指针
 - len：发送长度（字节）

 注意事项：
 - 使用 HAL_UART_Transmit 阻塞发送，超时固定 100ms；
 - 若发送大块数据（例如证书/固件分片），建议调整超时或改为 DMA/中断发送。
---------------------------------------------------------------------------*/
void at_send_raw(const uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0) return;
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100);
}

/*---------------------------------------------------------------------------
 Name        : at_register_urc
 Input       : const char *keyword, urc_callback_t callback
 Output      : None
 Description :
 注册一条 URC（异步上报）关键字匹配规则：当 `parse_rx_lines()` 解析到的某行文本
 包含 keyword 子串时，调用对应 callback(line)。

 输入参数：
 - keyword：关键字字符串（例如 "+MQTTURC:"、"+CEREG:" 等）；不能为空
 - callback：回调函数指针；不能为空

 行为说明：
 - 在 `s_urc_tbl[]` 中寻找空闲槽位写入 keyword/cb 并置 used=true；
 - 表满时静默失败（当前实现无返回值），必要时可扩展为返回错误码。

 注意事项：
 - 匹配方式为 strstr（子串匹配），keyword 需尽量具备唯一性，避免误匹配；
 - 回调执行发生在调用 `parse_rx_lines()` 的上下文中（通常为任务上下文），
   回调内应避免长时间阻塞以免影响命令处理时延。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : at_flush_rx
 Input       : None
 Output      : None
 Description :
 清空 AT 接收环形缓冲区：将读写指针复位为 0，丢弃当前已接收但未消费的所有数据。

 并发保护：
 - 通过关中断（PRIMASK）进入临界区，避免 ISR 在复位指针过程中并发写入导致指针不一致。

 使用场景：
 - 发送一条新的 AT 命令前清理历史残留（避免误把旧 OK/ERROR 识别为本次响应）。

 风险提示：
 - 清空会同时丢弃 URC（异步上报）数据；若系统依赖 URC 驱动状态机（如 MQTTURC 消息分发），
   频繁 flush 可能造成 URC 丢失，需要上层权衡或改进缓冲管理策略。
---------------------------------------------------------------------------*/
void at_flush_rx(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_rx_tail = 0;
    s_rx_write = 0;
    __set_PRIMASK(primask);
}

// ==================== UART模式切换 ====================

/*---------------------------------------------------------------------------
 Name        : uart_at_set_mode
 Input       : uart_at_mode_t mode, xmodem_byte_cb_t xmodem_cb
 Output      : None
 Description :
 切换 AT 串口框架的接收处理模式：
 - UART_AT_MODE_AT：按行解析 AT 响应/URC（OK/ERROR/URC）
 - UART_AT_MODE_XMODEM：将收到的每个字节直接回调给 XMODEM 接收器处理（不做 AT 行解析）

 输入参数：
 - mode：目标模式
 - xmodem_cb：XMODEM 模式下的字节回调；当 mode==UART_AT_MODE_XMODEM 时建议非空

 切换时的缓冲区处理（按当前实现）：
 - 切换到 XMODEM：不清空缓冲区（可能需要保留上一阶段尾部数据，具体取决于协议切换点）
 - 切换回 AT：清空读写指针，避免残留的 XMODEM 原始数据被当作 AT 文本解析

 并发保护：
 - 通过关中断（PRIMASK）在临界区内更新 `s_uart_mode`、`s_xmodem_byte_cb` 以及必要的指针复位，
   避免与 `HAL_UARTEx_RxEventCallback` 并发导致状态撕裂。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : uart_at_get_mode
 Input       : None
 Output      : uart_at_mode_t
 Description :
 获取当前 AT 串口框架的工作模式（AT 行解析 / XMODEM 原始字节回调）。

 返回值：
 - UART_AT_MODE_AT 或 UART_AT_MODE_XMODEM

 注意：
 - 返回值读取未加锁；`s_uart_mode` 为 volatile，读操作通常原子，但若系统对一致性要求极高，
   可在调用方侧关中断或增加轻量同步。
---------------------------------------------------------------------------*/
uart_at_mode_t uart_at_get_mode(void) {
    return s_uart_mode;
}
