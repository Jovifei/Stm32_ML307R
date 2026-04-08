# ML307R 4G 主MCU直控AT命令重构设计

**日期**: 2026-04-02  
**目标工程**: `D:\Document_Ref\ML307R_4G_MCU\stm32_4g_ref`  
**参考工程**: `D:\Document_Ref\ML307R_4G_AT`（ESP32桥接固件，AT命令序列参考）  
**架构参考**: `D:\Document_Ref\Reference_Projects\Stm32-MQTT-sim900a`（MCU全控模式）

---

## 背景与问题

当前 `stm32_4g_ref` 使用自定义文本协议（`device`/`get_down` 等命令）与 ML307R 通信，依赖 `ML307R_4G_AT`（ESP32固件）在上电时自主完成网络注册和MQTT连接，主MCU无法控制配网流程。

实际硬件中无ESP32芯片，STM32H7 USART2 直接连接 ML307R 的AT串口。需改为主MCU通过标准AT命令全控ML307R，逻辑参考ESP32工程实现（方案B：UART IDLE中断 + AT解析器 + FreeRTOS队列/信号量）。

---

## 架构设计

### 整体数据流

```
ML307R UART RX → [IDLE中断 + DMA] → [RX环形缓冲区]
                                            ↓
                                     [at_parser]
                                      /        \
                          OK/ERROR响应          URC主动上报
                               ↓                    ↓
                        释放cmd_semaphore    URC回调分发
                        (uart_at_send_cmd)   (mqtt_client/ml307r_init注册)

Cloud_Task:
  1. ml307r_init()        → 阻塞等网络就绪（信号量）
  2. mqtt_client_connect()→ 阻塞等MQTT连接（信号量）
  3. while(1): mqtt_publish() + 处理下行消息回调
```

### FreeRTOS 任务结构（保持不变）

| 任务 | 优先级 | 变化 |
|------|--------|------|
| Cloud_Task | Normal+5 | 重写：AT命令初始化+MQTT |
| UI_Task | Normal+4 | 不变（LVGL） |
| OTA_Task | Normal+2 | 轻微调整（OTA触发来源改变） |

---

## 文件变更

### 删除
- `Core/Src/cmd_parser.c` + `Core/Inc/cmd_parser.h` — 自定义文本协议，废弃
- `Core/Src/uart_ml307r.c` + `Core/Inc/uart_ml307r.h` — 替换为 uart_at
- `Core/Src/mcu_bridge.c` + `Core/Inc/mcu_bridge.h` — ESP32桥接，无此芯片

### 新增（移植自 ML307R_4G_AT）
- `Core/Src/uart_at.c` + `Core/Inc/uart_at.h`
- `Core/Src/at_parser.c` + `Core/Inc/at_parser.h`
- `Core/Src/ml307r_init.c` + `Core/Inc/ml307r_init.h`
- `Core/Src/mqtt_client.c` + `Core/Inc/mqtt_client.h`

### 重写
- `Core/Src/task_cloud.c` + `Core/Inc/task_cloud.h`

### 更新
- `Core/Inc/config.h` — MQTT配置改为ML307R_4G_AT里的配置
- `Core/Src/app_main.c` — 初始化调用改为 uart_at_init

---

## 模块设计

### uart_at.c/h — UART AT底层驱动

**职责**: UART收发、IDLE中断触发解析、URC回调注册

```c
// 初始化：启用IDLE中断 + DMA RX循环模式
void uart_at_init(void);

// 发送AT命令，阻塞等待期望响应（通过FreeRTOS信号量）
// 返回0=成功, -1=超时/错误
int uart_at_send_cmd(const char *cmd, const char *expected, uint32_t timeout_ms);

// 直接发送原始数据（不等响应）
void uart_at_send_raw(const uint8_t *data, uint16_t len);

// 注册URC前缀回调（如 "+MQTTURC"）
void uart_at_register_urc(const char *prefix, void (*callback)(const char *line));
```

**实现要点**:
- UART IDLE中断：将DMA缓冲区数据拷贝到环形缓冲区，通知 at_parser 任务
- `uart_at_send_cmd` 内部持互斥锁（同一时刻只有一条命令在途），等信号量
- at_parser 解析到 OK/ERROR 时释放信号量，解析到URC时调用注册的回调

### at_parser.c/h — AT响应解析器

**职责**: 从环形缓冲区逐行读取，区分命令响应与URC

```c
void at_parser_init(void);
void at_parser_process(void);  // 在专用任务或IDLE回调中调用
```

**解析规则**:
- `OK` / `ERROR` / `+CME ERROR` → 命令响应，释放 cmd_semaphore
- `+MQTTURC:` 开头 → MQTT下行URC，调用mqtt_client注册的回调
- `+CEREG:` / `+CREG:` → 网络状态URC，调用ml307r_init注册的回调
- 其余行 → 存入响应缓冲区供 uart_at_send_cmd 返回

### ml307r_init.c/h — ML307R初始化状态机

**职责**: 上电后完成网络注册到PDP激活，与ESP32工程 `ml307r_init.c` 逻辑相同

**AT命令序列**（移植自ML307R_4G_AT）:
```
1. AT          → 确认通信
2. ATE0        → 关闭回显
3. AT+CPIN?    → 检查SIM卡（等 READY）
4. AT+CEREG?   → 检查网络注册（等 0,1 或 0,5，最多重试30次，间隔1s）
5. AT+MIPCALL=1,1 → 激活PDP上下文（建立数据连接）
```

```c
typedef enum {
    ML307R_STATE_IDLE,
    ML307R_STATE_AT_TEST,
    ML307R_STATE_ECHO_OFF,
    ML307R_STATE_CHECK_SIM,
    ML307R_STATE_WAIT_REG,
    ML307R_STATE_ACTIVATE_PDP,
    ML307R_STATE_CONNECTED,
    ML307R_STATE_ERROR,
} ml307r_state_t;

// 阻塞执行初始化序列，成功返回0，失败返回-1
int ml307r_init(void);
ml307r_state_t ml307r_get_state(void);
```

### mqtt_client.c/h — MQTT AT命令客户端

**职责**: 使用ML307R内置MQTT AT命令完成连接/订阅/发布，处理下行URC

**AT命令序列**（ML307R特有，移植自ML307R_4G_AT）:
```
连接: AT+MQTTCONN="mqtt.cn.dream-maker.com",8883,"clientId","user","pass",1
订阅: AT+MQTTSUB="$sys/cn/dream-maker.com/cmd",1
发布: AT+MQTTPUB="$sys/cn/dream-maker.com/data",0,0,"payload"
断开: AT+MQTTDISC
URC: +MQTTURC: "message","topic","payload" → 回调处理
```

```c
// 连接并订阅，阻塞直到成功或超时
int mqtt_client_connect(void);

// 发布消息
int mqtt_client_publish(const char *topic, const char *payload);

// 注册下行消息回调
void mqtt_client_set_msg_callback(void (*cb)(const char *topic, const char *payload));

// 断开连接
void mqtt_client_disconnect(void);
```

**状态机**:
```
DISCONNECTED → CONNECTING → CONNECTED → [自动重连on断线URC]
```

### task_cloud.c — Cloud任务（重写）

**职责**: 调用各模块完成完整的连接+业务流程

```c
void Cloud_Task(void *argument) {
    osDelay(500);

    // 1. 网络初始化（MCU全控，阻塞）
    while (ml307r_init() != 0) {
        osDelay(5000);  // 失败则5s后重试
    }

    // 2. MQTT连接（MCU全控，阻塞）
    mqtt_client_set_msg_callback(on_cloud_message);
    while (mqtt_client_connect() != 0) {
        osDelay(3000);
    }

    // 3. 上报设备信息（对应ESP32的 device + mcu_version 命令）
    mqtt_publish_device_info();

    // 4. 主循环
    while (1) {
        mqtt_publish_data();     // 定时上报
        osDelay(200);
    }
}
```

### config.h — 配置更新

```c
// MQTT配置（来自ML307R_4G_AT/main.h）
#define MQTT_SERVER          "mqtt.cn.dream-maker.com"
#define MQTT_PORT            8883
#define MQTT_CLIENT_ID       "ml307r_client"
#define MQTT_USERNAME        "admin"
#define MQTT_PASSWORD        "public"
#define CMD_TOPIC            "$sys/cn/dream-maker.com/cmd"
#define DATA_TOPIC           "$sys/cn/dream-maker.com/data"
```

---

## 不变部分

- `task_ui.c/h` — LVGL显示，完全不改
- `task_ota.c/h` — OTA流程保留，触发来源由 `handle_update_fw` 改为 `on_cloud_message` 解析
- `flash_spi.c/h`、`xmodem.c/h` — Flash/XMODEM驱动不变
- `uart_debug.c/h` — Debug串口不变
- `main.c`、`app_main.c` — 微调初始化调用

---

## 关键差异：STM32 vs ESP32实现

| 方面 | ESP32 (ML307R_4G_AT) | STM32H7 (移植后) |
|------|---------------------|-----------------|
| UART驱动 | ESP-IDF UART driver | STM32 HAL + DMA + IDLE IRQ |
| 任务同步 | ESP-IDF EventGroup/Queue | FreeRTOS Semaphore/Queue |
| 延时 | `vTaskDelay` | `osDelay` (CMSIS-OS v2) |
| 内存 | heap_caps_malloc | pvPortMalloc / 静态缓冲区 |
| UART API | `uart_write_bytes` | `HAL_UART_Transmit` |
