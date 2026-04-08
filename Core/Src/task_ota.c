#include "task_ota.h"
#include "xmodem.h"
#include "flash_spi.h"
#include "config.h"
#include "cmsis_os2.h"
#include "at_parser.h"
#include "uart_at.h"
#include "app_main.h"
#include <string.h>
#include <stdio.h>

typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_WAIT_READY,
    OTA_STATE_RECEIVING,
    OTA_STATE_COMPLETE,
    OTA_STATE_BOOT
} ota_state_t;

static ota_state_t ota_state = OTA_STATE_IDLE;
static char ota_version[32];
static uint32_t ota_length;
static int ota_progress = 0;

bool ota_check_request(void) {
    return ota_state == OTA_STATE_WAIT_READY;
}

void ota_start_update(void) {
    if (ota_state != OTA_STATE_WAIT_READY) return;

    ota_state = OTA_STATE_RECEIVING;
    xmodem_init();

    // 切换UART到XMODEM模式，字节直接路由到xmodem_process_byte
    uart_at_set_mode(UART_AT_MODE_XMODEM, xmodem_process_byte);

    xmodem_start();
}

// MQTT下行消息回调（从Cloud_Task注册）
// 解析云端OTA命令: {"cmd":"ota_start"} 或 "update_fw <version> <length>"
static void on_cloud_mqtt_message(const char *topic, const char *payload, int len) {
    (void)topic;
    char buf[256];
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
    strncpy(buf, payload, len);
    buf[len] = '\0';

    // 尝试JSON格式: {"cmd":"ota_start"}
    char cmd[32] = {0};
    if (sscanf(buf, "{\"cmd\":\"%31[^\"]\"}", cmd) >= 1) {
        if (strcmp(cmd, "ota_start") == 0 || strcmp(cmd, "update") == 0) {
            ota_state = OTA_STATE_WAIT_READY;
        }
        return;
    }

    // 兼容旧格式: update_fw <version> <length>
    if (strncmp(buf, "update_fw", 9) == 0) {
        char version[32] = {0};
        uint32_t length = 0;
        if (sscanf(buf, "update_fw %31s %lu", version, &length) >= 2) {
            strncpy(ota_version, version, sizeof(ota_version) - 1);
            ota_length = length;
        }
        ota_state = OTA_STATE_WAIT_READY;
    }
}

// 注册MQTT消息回调（由Cloud_Task调用）
void ota_register_mqtt_callback(void) {
    at_mqtt_register_callback(on_cloud_mqtt_message);
}

void OTA_Task(void *argument) {
    (void)argument;

    // 等待Cloud_Task初始化完成（MQTT连接+回调注册）
    // 确保在MQTT消息回调链表就绪之前，OTA_Task不会处理任何消息
    osEventFlagsId_t flag = task_sync_get_cloud_ready_flag();
    if (flag != NULL) {
        osEventFlagsWait(flag, CLOUD_READY_FLAG_ID,
                         osFlagsWaitAll | osFlagsNoClear,
                         osWaitForever);
    }

    while (1) {
        switch (ota_state) {
        case OTA_STATE_IDLE:
            osDelay(100);
            break;

        case OTA_STATE_WAIT_READY:
            osDelay(50);
            break;

        case OTA_STATE_RECEIVING:
            ota_progress = xmodem_get_progress();
            if (xmodem_get_state() == XMODEM_COMPLETE) {
                ota_state = OTA_STATE_COMPLETE;
            } else if (xmodem_get_state() == XMODEM_ERROR) {
                // 切换回AT模式
                uart_at_set_mode(UART_AT_MODE_AT, NULL);
                ota_state = OTA_STATE_IDLE;
            }
            break;

        case OTA_STATE_COMPLETE:
            NVIC_SystemReset();
            break;

        default:
            osDelay(100);
            break;
        }
    }
}

int ota_get_progress(void) {
    return ota_progress;
}
