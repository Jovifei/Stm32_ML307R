#ifndef XMODEM_H
#define XMODEM_H

#include "stdint.h"
#include "stdbool.h"

// XMODEM块大小
#define XMODEM_BLOCK_SIZE 128

// XMODEM状态
typedef enum {
    XMODEM_IDLE = 0,
    XMODEM_RECEIVING,
    XMODEM_COMPLETE,
    XMODEM_ERROR
} xmodem_state_t;

// 初始化XMODEM接收
void xmodem_init(void);

// 开始接收
void xmodem_start(void);

// 停止接收
void xmodem_stop(void);

// 处理接收到的数据
void xmodem_process_byte(uint8_t byte);

// 获取状态
xmodem_state_t xmodem_get_state(void);

// 获取进度 (0-100)
int xmodem_get_progress(void);

#endif // XMODEM_H
