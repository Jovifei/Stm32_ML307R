#include "uart_debug.h"
#include <stdarg.h>
#include <stdio.h>

extern UART_HandleTypeDef DEBUG_UART;

void uart_debug_init(void) {
    // Debug UART已在CubeMX配置 (MX_USART1_UART_Init)
}

int uart_debug_printf(const char *format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)buf, len, 100);
    return len;
}
