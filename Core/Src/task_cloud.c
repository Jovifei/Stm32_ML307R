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

/*---------------------------------------------------------------------------
 Name        : publish_device_info
 Input       : None
 Output      : None
 Description :
 组织并上报设备基础信息到云端（通过 MQTT）。

 上报内容（当前实现）：
 - sn：设备序列号（`DEVICE_SN`）
 - model：产品型号（`DEVICE_PRODUCT_MODEL`）
 - fw_ver：MCU 固件版本（`MCU_FIRMWARE_VERSION`）

 发布主题：
 - 使用 `mqtt_client_publish(DATA_TOPIC, ...)` 发送到数据上报主题。

 设计意图：
 - 让云端在设备上线后尽快获取基本标识与版本信息；
 - 通常在 MQTT 连接建立、订阅回调注册完成后调用，避免上线阶段丢消息。
---------------------------------------------------------------------------*/
static void publish_device_info(void) {
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"sn\":\"%s\",\"model\":\"%s\",\"fw_ver\":\"%s\"}",
             DEVICE_SN, DEVICE_PRODUCT_MODEL, MCU_FIRMWARE_VERSION);
    mqtt_client_publish(DATA_TOPIC, payload, 0);
}

/*---------------------------------------------------------------------------
 Name        : Cloud_Task
 Input       : void *argument
 Output      : None
 Description :
 云通信主任务：负责 4G 模组入网、MQTT 连接建立、回调链路注册以及周期性在线上报。

 输入参数：
 - argument：RTOS 任务入口参数，当前未使用（显式丢弃）。

 任务阶段（按当前实现）：
 - 延时启动：上电后 `osDelay(500)`，为外设/模组稳定留出余量。
 - 阶段1：4G 入网拨号
   - 循环调用 `ml307r_init()`，失败则 5s 重试，直至成功。
 - 阶段2：MQTT 建链
   - 循环调用 `mqtt_client_connect()`，失败则 3s 重试，直至成功。
 - 阶段3：注册 MQTT 回调
   - 调用 `ota_register_mqtt_callback()` 注册 OTA 下行命令解析回调；
   - MCU 下行业务回调由 `MCU_Task_Init()` 内部注册（此处不重复注册）。
 - 阶段4：任务同步通知
   - 通过 `task_sync_get_cloud_ready_flag()` 获取事件标志组；
   - 设置 `CLOUD_READY_FLAG_ID`，通知 OTA/MCU 等任务可以开始业务运行。
 - 阶段5：上报设备信息
   - 调用 `publish_device_info()`。
 - 阶段6：主循环定时上报
   - 每 5s 上报一次在线心跳（当前示例为 `{"status":"online"}`）。

 注意事项：
 - 本任务采用“阻塞式重试”策略：在网络/MQTT未就绪前一直重试不退出；
 - 若要增强鲁棒性，可在失败时增加错误计数、回退策略或看门狗喂狗点。
---------------------------------------------------------------------------*/
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
