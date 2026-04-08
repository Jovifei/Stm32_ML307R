#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// ==================== 设备产品信息 ====================
#define DEVICE_PRODUCT_MODEL "DM-MIS800"
#define DEVICE_SN            "GTEST1000000011"
#define DEVICE_COUNTRY       "cn"
#define MCU_FIRMWARE_VERSION "V1.1.13"

// 产品凭证
#define DEVICE_PRODUCT_ID     "669f128f59b7727830b3b5fc"
#define DEVICE_PRODUCT_SECRET "WIr5vVBRHmURu8PB"

// ==================== API/HTTP服务器 ====================
#define API_SERVER          "api.dream-maker.com"
#define API_PORT            8443
#define PROVISION_PATH      "/APIServerV2/tool/mqtt/preset"
#define CERT_PATH           "/APIServerV2/tool/cert"

// ==================== 云端服务器 ====================
#define CLOUD_DOMAIN        "dream-maker.com"
#define MQTT_SERVER         "mqtt.cn.dream-maker.com"
#define MQTT_PORT           8883
#define CMD_TOPIC           "$sys/cn/dream-maker.com/cmd"
#define DATA_TOPIC          "$sys/cn/dream-maker.com/data"

// UART配置 (ML307R) - 使用 stm32_4g_ref 中的 UART2
#define ML307R_UART         huart2
#define ML307R_BAUD         115200

// 调试串口 - 使用 stm32_4g_ref 中的 UART1
#define DEBUG_UART          huart1

// 外部Flash SPI (需要在CubeMX中配置SPI1)
#define FLASH_SPI           hspi1
#define OTA_FLASH_ADDR      0x00000000
#define PROV_FLASH_ADDR     0x00100000  // 预配置信息存储地址(product_id/device_id/key等)

// FreeRTOS
#define TASK_STACK_SIZE     2048
#define CLOUD_TASK_PRIO     osPriority_t(osPriorityNormal + 5)
#define MCU_TASK_PRIO       osPriority_t(osPriorityNormal + 3)
#define UI_TASK_PRIO        osPriority_t(osPriorityNormal + 4)
#define OTA_TASK_PRIO       osPriority_t(osPriorityNormal + 2)

#endif // CONFIG_H