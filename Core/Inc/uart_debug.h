#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "stm32h7xx_hal.h"

void uart_debug_init(void);
int uart_debug_printf(const char *format, ...);

#endif // UART_DEBUG_H
