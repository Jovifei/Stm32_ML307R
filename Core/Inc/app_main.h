#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"

// 应用初始化 (由main.c的USER CODE调用)
void app_main_init(void);

// 获取Cloud_Task初始化完成事件标志
// OTA_Task调用此函数获取标志ID，然后等待 CLOUD_READY_FLAG_ID
osEventFlagsId_t task_sync_get_cloud_ready_flag(void);

#endif // APP_MAIN_H
