#ifndef TASK_OTA_H
#define TASK_OTA_H

#include "cmsis_os.h"
#include <stdbool.h>

void OTA_Task(void *argument);

// 检查是否有OTA请求
bool ota_check_request(void);

// 开始OTA更新
void ota_start_update(void);

// 获取OTA进度
int ota_get_progress(void);

// 注册MQTT消息回调（由Cloud_Task调用，接入云端OTA命令）
void ota_register_mqtt_callback(void);

#endif // TASK_OTA_H
