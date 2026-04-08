# ML307R 4G AT命令重构实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 stm32_4g_ref 的自定义文本协议替换为标准AT命令控制ML307R，参考ESP32工程架构（UART IDLE中断 + AT解析器 + FreeRTOS信号量），保留FreeRTOS+LVGL+OTA不变。

**Architecture:** MCU直接通过USART2发AT命令控制ML307R（ATE0→CPIN→CEREG→MIPCALL→MQTTCONN→MQTTSUB→MQTTPUB），用信号量同步命令响应，URC回调处理模块主动上报，Cloud_Task统一调度。

**Tech Stack:** STM32H7 + FreeRTOS + CMSIS-OS v2 + STM32 HAL + STM32CubeMX

---

## 文件结构映射

| 操作 | 路径 |
|------|------|
| **删除** | `Core/Inc/uart_ml307r.h`，`Core/Src/uart_ml307r.c`，`Core/Inc/cmd_parser.h`，`Core/Src/cmd_parser.c` |
| **新增** | `Core/Inc/uart_at.h`，`Core/Src/uart_at.c`，`Core/Inc/at_parser.h`，`Core/Src/at_parser.c`，`Core/Inc/ml307r_init.h`，`Core/Src/ml307r_init.c`，`Core/Inc/mqtt_client.h`，`Core/Src/mqtt_client.c`，`Core/Inc/at_config.h` |
| **重写** | `Core/Src/task_cloud.c`，`Core/Src/stm32h7xx_it.c`（加USART2中断），`Core/Inc/config.h` |
| **修改** | `Core/Src/app_main.c`，`Core/Inc/task_cloud.h`，`Core/Inc/task_ota.h`（OTA回调签名） |

---

## Task 1: 新增 at_config.h — AT超时与MQTT配置

**Files:**
- Create: `Core/Inc/at_config.h`

- [ ] **Step 1: 创建 at_config.h**

`Core/Inc/at_config.h` 内容：

```c
#ifndef AT_CONFIG_H
#define AT_CONFIG_H

// AT命令超时 (ms)
#define AT_TIMEOUT_DEFAULT  3000
#define AT_TIMEOUT_LONG     10000

// MQTT配置 (来自ML307R_4G_AT/main.h)
#define MQTT_SERVER          "mqtt.cn.dream-maker.com"
#define MQTT_PORT            8883
#define MQTT_CLIENT_ID       "ml307r_client"
#define MQTT_USERNAME        "admin"
#define MQTT_PASSWORD        "public"
#define CMD_TOPIC            "$sys/cn/dream-maker.com/cmd"
#define DATA_TOPIC           "$sys/cn/dream-maker.com/data"

// UART RX环形缓冲区大小
#define UART_AT_RX_BUF_SIZE  512

#endif // AT_CONFIG_H
```

- [ ] **Step 2: Commit**

```bash
git add Core/Inc/at_config.h
git commit -m "feat: add AT timeout and MQTT configuration header"
```

---

## Task 2: 新增 uart_at.c/h — UART AT底层驱动

**Files:**
- Create: `Core/Inc/uart_at.h`
- Create: `Core/Src/uart_at.c`
- Modify: `Core/Src/stm32h7xx_it.c`（在 USER CODE BEGIN 1 区域添加 USART2_IRQHandler）

- [ ] **Step 1: 创建 Core/Inc/uart_at.h**

```c
#ifndef UART_AT_H
#define UART_AT_H

#include <stdint.h>
#include <stdbool.h>

// 初始化UART（启用IDLE中断，中断模式接收）
void uart_at_init(void);

// 发送AT命令并等待响应，返回0成功，-1超时，-2错误
int at_send_command(const char *cmd, const char *expected_ok, uint32_t timeout_ms, char *response, int resp_len);

// 发送原始数据（不等响应）
void at_send_raw(const uint8_t *data, uint16_t len);

// 注册URC回调，keyword匹配则触发callback
typedef void (*urc_callback_t)(const char *line);
void at_register_urc(const char *keyword, urc_callback_t callback);

// 清空RX缓冲区
void at_flush_rx(void);

#endif // UART_AT_H
```

- [ ] **Step 2: 创建 Core/Src/uart_at.c**

```c
#include "uart_at.h"
#include "at_config.h"
#include "config.h"
#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef ML307R_UART;

// ==================== 静态变量 ====================

// RX环形缓冲区
static uint8_t s_rx_buf[UART_AT_RX_BUF_SIZE];
static uint16_t s_rx_head = 0;   // 写指针（中断写）
static uint16_t s_rx_tail = 0;   // 读指针（任务读）

// 同步信号量（命令发送后等响应）
static osSemaphoreId_t s_cmd_sem = NULL;

// 命令结果（OK=1, ERROR=0, TIMEOUT=-1）
static volatile int s_cmd_result = 0;

// URC回调表
#define URC_MAX 8
typedef struct { char keyword[24]; urc_callback_t cb; bool used; } urc_entry_t;
static urc_entry_t s_urc_tbl[URC_MAX];

// ==================== 内部函数 ====================

static void uart_at_irq_handler(void);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2) {
        for (uint16_t i = 0; i < Size; i++) {
            uint8_t ch = (uint8_t)(huart->Instance->RDR & 0xFF);
            uint16_t next = (s_rx_head + 1) % UART_AT_RX_BUF_SIZE;
            if (next != s_rx_tail) {
                s_rx_buf[s_rx_head] = ch;
                s_rx_head = next;
            }
        }
        HAL_UARTEx_ReceiveToIdle_IT(&ML307R_UART, NULL, 0);
    }
}

static void uart_at_parse_from_isr(void) {
    char line[256];
    uint16_t i = 0;
    bool in_line = false;

    while (s_rx_tail != s_rx_head) {
        uint8_t ch = s_rx_buf[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) % UART_AT_RX_BUF_SIZE;

        if (ch == '\r' || ch == '\n') {
            if (in_line && i > 0) {
                line[i] = '\0';
                in_line = false;

                if (strstr(line, "\r\nOK") != NULL || strcmp(line, "OK") == 0) {
                    s_cmd_result = 1;
                    osSemaphoreRelease(s_cmd_sem);
                } else if (strstr(line, "ERROR") != NULL || strstr(line, "+CME ERROR") != NULL || strstr(line, "+CMS ERROR") != NULL) {
                    s_cmd_result = 0;
                    osSemaphoreRelease(s_cmd_sem);
                } else {
                    for (int j = 0; j < URC_MAX; j++) {
                        if (s_urc_tbl[j].used && strstr(line, s_urc_tbl[j].keyword) != NULL) {
                            if (s_urc_tbl[j].cb) s_urc_tbl[j].cb(line);
                            break;
                        }
                    }
                }
                i = 0;
            }
        } else if (i < sizeof(line) - 1) {
            line[i++] = (char)ch;
            in_line = true;
        }
    }
}

// ==================== 公开API ====================

void uart_at_init(void) {
    s_rx_head = 0;
    s_rx_tail = 0;
    s_cmd_result = 0;

    // 创建二值信号量
    osSemaphoreAttr_t sem_attr = { .name = "at_cmd" };
    s_cmd_sem = osSemaphoreNew(1, 0, &sem_attr);

    // 初始化URC表
    memset(s_urc_tbl, 0, sizeof(s_urc_tbl));

    // 启动IDLE中断接收（首次）
    HAL_UARTEx_ReceiveToIdle_IT(&ML307R_UART, NULL, 0);
}

int at_send_command(const char *cmd, const char *expected_ok, uint32_t timeout_ms, char *response, int resp_len) {
    if (cmd == NULL) return -2;

    at_flush_rx();

    char cmd_buf[320];
    snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);
    HAL_UART_Transmit(&ML307R_UART, (uint8_t *)cmd_buf, strlen(cmd_buf), 100);

    s_cmd_result = 0;
    if (osSemaphoreAcquire(s_cmd_sem, timeout_ms) != osOK) {
        return -1; // timeout
    }

    if (response != NULL && resp_len > 0) {
        uint16_t copied = 0;
        while (s_rx_tail != s_rx_head && copied < (uint16_t)(resp_len - 1)) {
            response[copied++] = s_rx_buf[s_rx_tail];
            s_rx_tail = (s_rx_tail + 1) % UART_AT_RX_BUF_SIZE;
        }
        response[copied] = '\0';
    }

    return (s_cmd_result == 1) ? 0 : -2;
}

void at_send_raw(const uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0) return;
    HAL_UART_Transmit(&ML307R_UART, (uint8_t *)data, len, 100);
}

void at_register_urc(const char *keyword, urc_callback_t callback) {
    if (keyword == NULL || callback == NULL) return;
    for (int i = 0; i < URC_MAX; i++) {
        if (!s_urc_tbl[i].used) {
            strncpy(s_urc_tbl[i].keyword, keyword, sizeof(s_urc_tbl[i].keyword) - 1);
            s_urc_tbl[i].cb = callback;
            s_urc_tbl[i].used = true;
            break;
        }
    }
}

void at_flush_rx(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_rx_head = 0;
    s_rx_tail = 0;
    __set_PRIMASK(primask);
}
```

- [ ] **Step 3: 修改 stm32h7xx_it.c，在 USER CODE BEGIN 1 添加USART2中断处理**

在 `/* USER CODE BEGIN 1 */` 和 `/* USER CODE END 1 */` 之间添加：

```c
/* USER CODE BEGIN 1 */
extern void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&ML307R_UART);
}
/* USER CODE END 1 */
```

同时在文件顶部（已有 `#include "main.h"`）后面确认有 `#include "config.h"`，USART2 中断默认在 startup 文件中已启用。

- [ ] **Step 4: Commit**

```bash
git add Core/Inc/uart_at.h Core/Src/uart_at.c Core/Src/stm32h7xx_it.c
git commit -m "feat: add uart_at driver with IDLE interrupt, ring buffer, semaphore sync"
```

---

## Task 3: 新增 ml307r_init.c/h — ML307R网络初始化

**Files:**
- Create: `Core/Inc/ml307r_init.h`
- Create: `Core/Src/ml307r_init.c`

- [ ] **Step 1: 创建 Core/Inc/ml307r_init.h**

```c
#ifndef ML307R_INIT_H
#define ML307R_INIT_H

#include <stdbool.h>

typedef enum {
    ML307R_STATE_INIT = 0,
    ML307R_STATE_SIM_CHECK,
    ML307R_STATE_REGISTERED,
    ML307R_STATE_DIAL,
    ML307R_STATE_CONNECTED,
    ML307R_STATE_ERROR
} ml307r_state_t;

typedef struct {
    int rssi;
    int ber;
} signal_quality_t;

int ml307r_init(void);
ml307r_state_t ml307r_get_state(void);
int ml307r_get_signal_quality(signal_quality_t *sq);
bool ml307r_is_arrears(void);
int ml307r_reconnect(void);

#endif // ML307R_INIT_H
```

- [ ] **Step 2: 创建 Core/Src/ml307r_init.c**

```c
#include "ml307r_init.h"
#include "uart_at.h"
#include "at_config.h"
#include "cmsis_os2.h"
#include <string.h>

static ml307r_state_t s_state = ML307R_STATE_INIT;
static signal_quality_t s_sq = {99, 99};
static osMutexId_t s_mutex = NULL;

int ml307r_init(void) {
    int ret;
    char resp[128];

    if (s_mutex == NULL) {
        s_mutex = osMutexNew(NULL);
    }

    osMutexAcquire(s_mutex, osWaitForever);
    s_state = ML307R_STATE_INIT;

    // 1. AT通信测试
    s_state = ML307R_STATE_INIT;
    ret = at_send_command("AT", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) { s_state = ML307R_STATE_ERROR; osMutexRelease(s_mutex); return -1; }

    // 2. 关闭回显 ATE0
    ret = at_send_command("ATE0", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) { s_state = ML307R_STATE_ERROR; osMutexRelease(s_mutex); return -1; }

    // 3. 检查SIM卡 AT+CPIN?
    s_state = ML307R_STATE_SIM_CHECK;
    ret = at_send_command("AT+CPIN?", "READY", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) { s_state = ML307R_STATE_ERROR; osMutexRelease(s_mutex); return -1; }

    // 4. 等待网络注册 AT+CEREG?（最多30次，每次1s）
    int retry = 0;
    while (retry < 30) {
        ret = at_send_command("AT+CEREG?", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
        if (ret == 0 && (strstr(resp, "+CEREG: 0,1") != NULL || strstr(resp, "+CEREG: 1,1") != NULL)) {
            break;
        }
        retry++;
        osDelay(1000);
    }
    if (retry >= 30) { s_state = ML307R_STATE_ERROR; osMutexRelease(s_mutex); return -1; }
    s_state = ML307R_STATE_REGISTERED;

    // 5. 激活PDP拨号 AT+MIPCALL=1,1
    s_state = ML307R_STATE_DIAL;
    ret = at_send_command("AT+MIPCALL=1,1", "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
    if (ret != 0) { s_state = ML307R_STATE_ERROR; osMutexRelease(s_mutex); return -1; }

    s_state = ML307R_STATE_CONNECTED;
    osMutexRelease(s_mutex);
    return 0;
}

ml307r_state_t ml307r_get_state(void) {
    osMutexAcquire(s_mutex, osWaitForever);
    ml307r_state_t st = s_state;
    osMutexRelease(s_mutex);
    return st;
}

int ml307r_get_signal_quality(signal_quality_t *sq) {
    if (sq == NULL) return -1;
    char resp[128];
    int rssi = 99, ber = 99;
    int ret = at_send_command("AT+CSQ", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret == 0) {
        sscanf(resp, "+CSQ: %d,%d", &rssi, &ber);
    }
    sq->rssi = rssi; sq->ber = ber;
    s_sq.rssi = rssi; s_sq.ber = ber;
    return 0;
}

bool ml307r_is_arrears(void) {
    signal_quality_t sq;
    ml307r_get_signal_quality(&sq);
    ml307r_state_t st = ml307r_get_state();
    return (sq.rssi >= 10 && sq.rssi <= 31 &&
            st != ML307R_STATE_REGISTERED &&
            st != ML307R_STATE_DIAL &&
            st != ML307R_STATE_CONNECTED);
}

int ml307r_reconnect(void) {
    char resp[128];
    int ret;

    at_send_command("AT+CFUN=0", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    osDelay(500);
    at_send_command("AT+CFUN=1", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    osDelay(2000);

    int retry = 0;
    while (retry < 30) {
        ret = at_send_command("AT+CEREG?", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
        if (ret == 0 && (strstr(resp, "+CEREG: 0,1") != NULL || strstr(resp, "+CEREG: 1,1") != NULL)) {
            break;
        }
        retry++;
        osDelay(1000);
    }
    if (retry >= 30) { s_state = ML307R_STATE_ERROR; return -1; }

    s_state = ML307R_STATE_REGISTERED;
    ret = at_send_command("AT+MIPCALL=1,1", "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
    if (ret != 0) { s_state = ML307R_STATE_ERROR; return -1; }

    s_state = ML307R_STATE_CONNECTED;
    return 0;
}
```

- [ ] **Step 3: Commit**

```bash
git add Core/Inc/ml307r_init.h Core/Src/ml307r_init.c
git commit -m "feat: add ml307r_init network initialization state machine"
```

---

## Task 4: 新增 at_parser.c/h — AT MQTT命令封装

**Files:**
- Create: `Core/Inc/at_parser.h`
- Create: `Core/Src/at_parser.c`

- [ ] **Step 1: 创建 Core/Inc/at_parser.h**

```c
#ifndef AT_PARSER_H
#define AT_PARSER_H

#include <stdint.h>
#include <stdbool.h>

typedef void (*mqtt_msg_callback_t)(const char *topic, const char *payload, int payload_len);

int at_mqtt_config(const char *host, int port, const char *client_id,
                   const char *username, const char *password);
int at_mqtt_connect(void);
int at_mqtt_disconnect(void);
int at_mqtt_subscribe(const char *topic, int qos);
int at_mqtt_publish(const char *topic, int qos, const char *data, int data_len);
void at_mqtt_register_callback(mqtt_msg_callback_t callback);

#endif // AT_PARSER_H
```

- [ ] **Step 2: 创建 Core/Src/at_parser.c**

```c
#include "at_parser.h"
#include "uart_at.h"
#include "at_config.h"
#include <string.h>
#include <stdio.h>

#define MQTT_CONN_ID 0

static mqtt_msg_callback_t s_msg_cb = NULL;
static bool s_urc_registered = false;

static void mqtt_urc_handler(const char *line) {
    if (line == NULL) return;

    if (strstr(line, "+MQTTURC: \"conn\",0,0") != NULL) {
        return;
    }

    if (strstr(line, "+MQTTURC: \"suback\"") != NULL) {
        return;
    }

    const char *prefix = "+MQTTURC: \"message\"";
    if (strncmp(line, prefix, strlen(prefix)) == 0 && s_msg_cb != NULL) {
        char topic[128] = {0};
        char payload[512] = {0};
        int qos = 0, len = 0;
        if (sscanf(line, "+MQTTURC: \"message\",\"%127[^\"]\",%d,%d,\"%511[^\"]\"",
                   topic, &qos, &len, payload) >= 4) {
            s_msg_cb(topic, payload, len < 512 ? len : 511);
        }
    }
}

int at_mqtt_config(const char *host, int port, const char *client_id,
                    const char *username, const char *password) {
    char cmd[512], resp[256];
    snprintf(cmd, sizeof(cmd),
             "AT+MQTTCONN=%d,\"%s\",%d,\"%s\",\"%s\",\"%s\"",
             MQTT_CONN_ID, host, port, client_id, username, password);
    return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

int at_mqtt_connect(void) {
    char cmd[64], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=%d", MQTT_CONN_ID);
    return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

int at_mqtt_disconnect(void) {
    char cmd[64], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTDISC=%d", MQTT_CONN_ID);
    return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

int at_mqtt_subscribe(const char *topic, int qos) {
    char cmd[256], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=%d,\"%s\",%d", MQTT_CONN_ID, topic, qos);
    return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

int at_mqtt_publish(const char *topic, int qos, const char *data, int data_len) {
    if (data == NULL || data_len == 0) data_len = strlen(data);
    char cmd[512], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTPUB=%d,\"%s\",%d,0,0,%d,\"%s\"",
             MQTT_CONN_ID, topic, qos, data_len, data);
    return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

void at_mqtt_register_callback(mqtt_msg_callback_t callback) {
    s_msg_cb = callback;
    if (callback != NULL && !s_urc_registered) {
        at_register_urc("+MQTTURC:", mqtt_urc_handler);
        s_urc_registered = true;
    }
}
```

- [ ] **Step 3: Commit**

```bash
git add Core/Inc/at_parser.h Core/Src/at_parser.c
git commit -m "feat: add at_parser MQTT AT command封装"
```

---

## Task 5: 新增 mqtt_client.c/h — MQTT客户端状态机

**Files:**
- Create: `Core/Inc/mqtt_client.h`
- Create: `Core/Src/mqtt_client.c`
- Modify: `Core/Src/task_cloud.c`（在Task 7中重写）

- [ ] **Step 1: 创建 Core/Inc/mqtt_client.h**

```c
#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

typedef enum {
    MQTT_STATE_DISCONNECTED = 0,
    MQTT_STATE_CONNECTING,
    MQTT_STATE_CONNECTED,
    MQTT_STATE_ERROR
} mqtt_state_t;

int mqtt_client_connect(void);
int mqtt_client_disconnect(void);
int mqtt_client_publish(const char *topic, const char *payload, int qos);
mqtt_state_t mqtt_client_get_state(void);

#endif // MQTT_CLIENT_H
```

- [ ] **Step 2: 创建 Core/Src/mqtt_client.c**

```c
#include "mqtt_client.h"
#include "at_parser.h"
#include "at_config.h"
#include "cmsis_os2.h"
#include <string.h>

static mqtt_state_t s_state = MQTT_STATE_DISCONNECTED;
static osMutexId_t s_mutex = NULL;

static void on_mqtt_message(const char *topic, const char *payload, int len) {
    // TODO: 云端命令处理（目前占位，后续接入业务逻辑）
}

int mqtt_client_connect(void) {
    if (s_mutex == NULL) {
        s_mutex = osMutexNew(NULL);
    }

    osMutexAcquire(s_mutex, osWaitForever);

    at_mqtt_register_callback(on_mqtt_message);

    int ret = at_mqtt_config(MQTT_SERVER, MQTT_PORT,
                              MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
    if (ret != 0) { osMutexRelease(s_mutex); return -1; }

    ret = at_mqtt_connect();
    if (ret != 0) { osMutexRelease(s_mutex); return -1; }

    ret = at_mqtt_subscribe(CMD_TOPIC, 1);
    if (ret != 0) { osMutexRelease(s_mutex); return -1; }

    s_state = MQTT_STATE_CONNECTED;
    osMutexRelease(s_mutex);
    return 0;
}

int mqtt_client_disconnect(void) {
    osMutexAcquire(s_mutex, osWaitForever);
    int ret = at_mqtt_disconnect();
    s_state = MQTT_STATE_DISCONNECTED;
    osMutexRelease(s_mutex);
    return ret;
}

int mqtt_client_publish(const char *topic, const char *payload, int qos) {
    return at_mqtt_publish(topic, qos, payload, 0);
}

mqtt_state_t mqtt_client_get_state(void) {
    osMutexAcquire(s_mutex, osWaitForever);
    mqtt_state_t st = s_state;
    osMutexRelease(s_mutex);
    return st;
}
```

- [ ] **Step 3: Commit**

```bash
git add Core/Inc/mqtt_client.h Core/Src/mqtt_client.c
git commit -m "feat: add mqtt_client state machine wrapper"
```

---

## Task 6: 重写 task_cloud.c — 云端通信任务

**Files:**
- Modify: `Core/Src/task_cloud.c`（完整重写）
- Modify: `Core/Inc/task_cloud.h`（仅更新注释）
- Modify: `Core/Inc/config.h`（更新MQTT配置）

- [ ] **Step 1: 更新 Core/Inc/config.h 的MQTT配置部分**

将 config.h 中 MQTT_SERVER/MQTT_PORT 两行替换为：

```c
// MQTT配置 (来自ML307R_4G_AT)
#define MQTT_SERVER         "mqtt.cn.dream-maker.com"
#define MQTT_PORT           8883
#define MQTT_CLIENT_ID      "ml307r_client"
#define MQTT_USERNAME       "admin"
#define MQTT_PASSWORD       "public"
#define CMD_TOPIC           "$sys/cn/dream-maker.com/cmd"
#define DATA_TOPIC          "$sys/cn/dream-maker.com/data"
```

同时删除 DEVICE_PRODUCT_ID、DEVICE_PRODUCT_SECRET、DEVICE_PRODUCT_MODEL（已不在ML307R AT方案中使用）。

- [ ] **Step 2: 重写 Core/Src/task_cloud.c**

```c
#include "task_cloud.h"
#include "ml307r_init.h"
#include "mqtt_client.h"
#include "at_config.h"
#include "config.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

// FreeRTOS任务属性
#define CLOUD_TASK_STACK    2048
#define CLOUD_TASK_PRIO     osPriorityNormal + 5

static osThreadId_t s_cloud_task_handle = NULL;

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

    // 阶段3: 上报设备信息
    publish_device_info();

    // 阶段4: 主循环
    while (1) {
        mqtt_client_publish(DATA_TOPIC, "{\"status\":\"online\"}", 0);
        osDelay(5000);
    }
}
```

- [ ] **Step 3: 更新 Core/Inc/task_cloud.h**

```c
#ifndef TASK_CLOUD_H
#define TASK_CLOUD_H

void Cloud_Task(void *argument);

#endif // TASK_CLOUD_H
```

- [ ] **Step 4: Commit**

```bash
git add Core/Src/task_cloud.c Core/Inc/task_cloud.h Core/Inc/config.h
git commit -m "refactor: rewrite task_cloud with AT command control flow"
```

---

## Task 7: 删除废弃文件 & 更新 app_main.c

**Files:**
- Delete: `Core/Inc/uart_ml307r.h`，`Core/Src/uart_ml307r.c`，`Core/Inc/cmd_parser.h`，`Core/Src/cmd_parser.c`
- Modify: `Core/Src/app_main.c`

- [ ] **Step 1: 更新 app_main.c 替换初始化调用**

将：
```c
#include "uart_ml307r.h"
#include "cmd_parser.h"
```

替换为：
```c
#include "uart_at.h"
#include "ml307r_init.h"
```

将 `uart_ml307r_init();` 替换为 `uart_at_init();`
删除 `cmd_parser_init();`

- [ ] **Step 2: 删除废弃文件**

```bash
rm Core/Inc/uart_ml307r.h Core/Src/uart_ml307r.c Core/Inc/cmd_parser.h Core/Src/cmd_parser.c
git add -A
git commit -m "refactor: remove deprecated uart_ml307r and cmd_parser"
```

---

## Task 8: OTA回调接入（适配MQTT URC触发）

**Files:**
- Modify: `Core/Src/task_ota.c`（在USER CODE BEGIN区域添加MQTT消息回调）
- Modify: `Core/Inc/task_ota.h`（新增 `ota_on_cloud_message`）

- [ ] **Step 1: 在 task_ota.c 中注册MQTT下行消息回调**

在 `task_ota.c` 的 USER CODE BEGIN 区域（或文件顶部）添加：

```c
// 在ota_start之后调用
void ota_register_mqtt_trigger(void) {
    at_mqtt_register_callback(ota_on_cloud_message);
}

// 云端MQTT消息处理入口
static void ota_on_cloud_message(const char *topic, const char *payload, int len) {
    (void)topic;
    char buf[512];
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    strncpy(buf, payload, len);
    buf[len] = '\0';
    // 解析 {"cmd":"update","version":"x.x.x","url":"..."}
    char cmd[32] = {0}, version[32] = {0}, url[256] = {0};
    if (sscanf(buf, "{\"cmd\":\"%31[^\"]\",\"version\":\"%31[^\"]\",\"url\":\"%255[^\"]\"}",
               cmd, version, url) >= 3) {
        if (strcmp(cmd, "update") == 0) {
            http_ota_start(url);
        }
    }
}
```

同时在 task_cloud.c 的 MQTT 连接成功后调用 `ota_register_mqtt_trigger()`。

- [ ] **Step 2: Commit**

```bash
git add Core/Src/task_ota.c Core/Src/task_cloud.c
git commit -m "feat: connect OTA trigger to MQTT downstream messages"
```

---

## 自检清单

- [x] Task 1: at_config.h — AT超时和MQTT配置
- [x] Task 2: uart_at.c/h — UART驱动 + IDLE中断 + 信号量同步
- [x] Task 3: ml307r_init.c/h — 网络初始化状态机（完整5步）
- [x] Task 4: at_parser.c/h — MQTT AT命令封装（config/connect/subscribe/publish/URC）
- [x] Task 5: mqtt_client.c/h — MQTT客户端状态机
- [x] Task 6: task_cloud.c — 重写云端任务，AT全控流程
- [x] Task 7: 删除废弃文件，app_main.c更新
- [x] Task 8: OTA接入MQTT URC触发

**类型一致性检查:**
- `ml307r_init()` 返回 `int`（0成功/-1失败）— 所有调用处一致
- `at_send_command()` 返回 `int`（0成功/-1超时/-2错误）— ml307r_init 和 at_parser 一致
- `mqtt_client_connect()` 返回 `int`（0成功/-1失败）— task_cloud 逻辑一致
- `mqtt_state_t` 枚举值与 mqtt_client.c 定义一致

**Spec覆盖检查:**
- UART IDLE中断接收 → Task 2 ✓
- AT解析器（OK/ERROR/URC）→ Task 2 ✓
- ML307R初始化（ATE0→CPIN→CEREG→MIPCALL）→ Task 3 ✓
- MQTT AT命令（+MQTTCONN/SUB/PUB）→ Task 4 ✓
- Cloud_Task MCU全控流程 → Task 6 ✓
- 配置改为ML307R_4G_AT配置 → Task 1+6 ✓
- 保留FreeRTOS+LVGL+OTA → Task 7+8 ✓
