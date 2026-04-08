#include "task_cloud.h"
#include "ml307r_init.h"
#include "mqtt_client.h"
#include "task_ota.h"
#include "task_mcu.h"
#include "app_main.h"
#include "at_config.h"
#include "config.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

#define CLOUD_TASK_STACK    2048
#define CLOUD_TASK_PRIO     osPriorityNormal + 5

static void publish_device_info(void) {
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"sn\":\"%s\",\"model\":\"%s\",\"fw_ver\":\"%s\"}",
             DEVICE_SN, DEVICE_PRODUCT_MODEL, MCU_FIRMWARE_VERSION);
    mqtt_client_publish(DATA_TOPIC, payload, 0);
}

void Cloud_Task(void *argument) {
    (void)argument;
    osDelay(500);

    // 阶段1: ML307R网络初始化（MCU全控）
    while (1) {
        if (ml307r_init() == 0) break;
        osDelay(5000);
    }

    // 阶段2: MQTT连接（MCU全控）
    while (1) {
        if (mqtt_client_connect() == 0) break;
        osDelay(3000);
    }

    // 阶段3: 注册MQTT回调（OTA命令解析 + MCU消息处理）
    ota_register_mqtt_callback();
    // 注意: MCU消息回调由MCU_Task内部注册，无需在此处调用

    // 通知Cloud初始化完成，OTA_Task可以开始运行
    osEventFlagsId_t flag = task_sync_get_cloud_ready_flag();
    if (flag != NULL) {
        osEventFlagsSet(flag, CLOUD_READY_FLAG_ID);
    }

    // 阶段4: 上报设备信息
    publish_device_info();

    // 阶段5: 主循环 - 定时上报
    while (1) {
        mqtt_client_publish(DATA_TOPIC, "{\"status\":\"online\"}", 0);
        osDelay(5000);
    }
}
