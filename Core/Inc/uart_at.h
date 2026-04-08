#ifndef UART_AT_H
#define UART_AT_H

#include <stdint.h>
#include <stdbool.h>

// 初始化UART（启用IDLE中断，中断模式接收）
void uart_at_init(void);

// 发送AT命令并等待OK返回，返回0成功，-1超时，-2错误
int at_send_command(const char *cmd, const char *expected_ok, uint32_t timeout_ms, char *response, int resp_len);

// 发送原始数据（不等响应）
void at_send_raw(const uint8_t *data, uint16_t len);

// 注册URC回调，keyword匹配则触发callback
typedef void (*urc_callback_t)(const char *line);
void at_register_urc(const char *keyword, urc_callback_t callback);

// 清空RX缓冲区
void at_flush_rx(void);

// ==================== UART模式切换 ====================
// UART模式：AT命令模式 vs XMODEM原始数据模式
typedef enum {
    UART_AT_MODE_AT = 0,      // AT命令模式（默认）
    UART_AT_MODE_XMODEM       // XMODEM原始数据模式
} uart_at_mode_t;

// XMODEM字节回调类型
typedef void (*xmodem_byte_cb_t)(uint8_t byte);

// 设置UART工作模式
// AT模式：字节进入环形缓冲区，由AT解析器处理
// XMODEM模式：字节直接回调给xmodem_byte_cb，不经过AT解析器
void uart_at_set_mode(uart_at_mode_t mode, xmodem_byte_cb_t xmodem_cb);

// 获取当前UART模式
uart_at_mode_t uart_at_get_mode(void);

#endif // UART_AT_H
