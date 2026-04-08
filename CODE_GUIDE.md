# 工程代码阅读指南 — STM32H7 + ML307R 4G IoT

## 1. 项目概览

| 项目 | 说明 |
|------|------|
| **芯片** | STM32H7 (Cortex-M7) |
| **4G模块** | ML307R（移远/中移物联网模块） |
| **通信方式** | AT命令 + UART2 (115200bps) |
| **调试串口** | UART1 (115200bps) |
| **操作系统** | FreeRTOS + CMSIS-OS v2 |
| **云端平台** | Dream Maker Cloud (`mqtt.cn.dream-maker.com:8883`) |
| **协议** | MQTT over TLS |
| **OTA方式** | XMODEM-CRC16（固件写入外部SPI Flash） |
| **固件版本** | `1.0.0` |

## 2. 硬件资源映射

```
STM32H7                    ML307R 4G模块
  TX(PA2)  ──────────────── RX
  RX(PA3)  ──────────────── TX
  GND      ──────────────── GND

STM32H7                    W25Qxx SPI Flash
  SPI1_SCK  ─────────────── CLK
  SPI1_MOSI ─────────────── DI
  SPI1_MISO ─────────────── DO
  PB14      ─────────────── CS
  GND       ─────────────── GND

STM32H7                    PC (调试)
  USART1 TX(PA9)  ───────── RX
  USART1 RX(PA10) ───────── TX
```

## 3. 目录结构

```
Core/
├── Src/                          # 应用层源码
│   ├── main.c                    # 入口：时钟/UART初始化，osKernelStart
│   ├── app_main.c                # 应用任务创建 + Cloud/OTA同步
│   ├── task_cloud.c              # Cloud_Task：ML307R初始化 + MQTT + 心跳
│   ├── task_ota.c                 # OTA_Task：XMODEM接收 + 固件烧写
│   ├── task_ui.c                 # UI_Task：LVGL界面（暂未启用）
│   ├── uart_at.c                 # UART2驱动：环形缓冲区 + IDLE中断 + AT解析
│   ├── at_parser.c               # AT命令封装 + MQTT URC回调分发
│   ├── mqtt_client.c             # MQTT连接状态管理
│   ├── ml307r_init.c             # ML307R初始化状态机（SIM/注册/PDP）
│   ├── xmodem.c                  # XMODEM接收器（CRC16 + ACK/NAK）
│   ├── flash_spi.c               # W25Qxx SPI Flash驱动
│   ├── uart_debug.c              # UART1调试打印
│   ├── freertos.c                # 空文件（CMSIS-OS占位）
│   ├── stm32h7xx_hal_msp.c       # HAL底层回调
│   └── stm32h7xx_it.c           # 中断向量表
└── Inc/                          # 头文件
    ├── config.h                  # 设备配置（SN/主题/服务器地址）
    ├── at_config.h               # AT超时/MQTT配置
    └── *.h                       # 各模块接口声明
```

## 4. 任务架构图

```
┌────────────────────────────────────────────────────────────────┐
│                      STM32H7 FreeRTOS                          │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   Cloud_Task    │  │    OTA_Task     │  │    UI_Task      │ │
│  │  (优先级+5)     │  │  (优先级+2)     │  │  (优先级+4)     │ │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘ │
│           │                    │                    │           │
│           └──────────┬─────────┴──────────┬─────────┘           │
│                      │                     │                     │
│                      ↓                     │                     │
│            ┌──────────────────┐            │                     │
│            │   mqtt_client    │            │                     │
│            │  (连接状态管理)   │            │                     │
│            └────────┬─────────┘            │                     │
│                     │                       │                     │
│                     ↓                       │                     │
│            ┌──────────────────┐              │                     │
│            │   at_parser      │←─────────────┘                     │
│            │  (URC回调链表)    │    注册 on_cloud_mqtt_message      │
│            └────────┬─────────┘                                    │
│                     │                                              │
│    ┌────────────────┼────────────────────────────────┐             │
│    │                ↓                                │             │
│    │  ┌─────────────────────┐  ┌──────────────────┐  │             │
│    │  │ UART_AT_MODE_AT     │  │ UART_AT_MODE_     │  │             │
│    │  │ (环形缓冲区 → 解析)  │  │ XMODEM            │  │             │
│    │  │                     │  │ (字节直接回调)     │  │             │
│    │  └─────────────────────┘  └────────┬─────────┘  │             │
│    │                                       │           │             │
│    └───────────────┬───────────────────────┘           │             │
│                    ↓                                    │             │
│              USART2 DMA+IDLE                            │             │
│                   │                                     │             │
│                   └──────────────┬────────────────────┘             │
│                                  ↓                                    │
│                           ML307R 4G模块                              │
│                                  ↓                                    │
│                        Dream Maker Cloud (MQTT:8883 TLS)            │
└────────────────────────────────────────────────────────────────┘
```

## 5. 核心模块详解

### 5.1 `uart_at.c` — UART2 + AT命令通信层

**这是整个工程最核心的底层模块。**

```
职责：STM32 <-> ML307R 之间的所有串口通信

核心数据结构 — 环形缓冲区（双指针）:
  s_rx_buf[512]    接收缓冲区
  s_rx_tail        读指针（任务消费）
  s_rx_write       写指针（ISR写入）

UART工作模式:
  UART_AT_MODE_AT     → AT命令模式：字节进入环形缓冲区，由AT解析器按行处理
  UART_AT_MODE_XMODEM → XMODEM模式：字节直接回调给xmodem_process_byte()

关键函数:
  uart_at_init()              初始化环形缓冲区、信号量、启动IDLE中断
  at_send_command(cmd, ...)   发送AT命令，等待OK/ERROR响应（信号量同步）
  at_send_raw(data, len)      发送原始数据（无响应等待）
  at_register_urc(keyword, cb) 注册URC关键字回调
  at_flush_rx()               清空环形缓冲区
  uart_at_set_mode(mode, cb)  切换UART模式（AT ↔ XMODEM）
```

**AT命令发送流程：**
```
at_send_command("AT+CPIN?")
  → 清空缓冲区
  → 发送 "AT+CPIN?\r\n"
  → 等待信号量（超时则返回-1）
  → 复制响应到response缓冲区
  → 返回0(成功) / -1(超时) / -2(错误)
```

**URC处理流程：**
```
USART2_IDLE_IRQ
  → HAL_UARTEx_RxEventCallback
  → 更新s_rx_write
  → parse_rx_lines() 扫描环形缓冲区
  → 按行分割，检测OK/ERROR/已注册URC关键字
  → 触发对应回调（如mqtt_urc_handler）
```

### 5.2 `at_parser.c` — AT命令封装 + MQTT URC

**职责：将UART层的原始AT指令封装为易用API，并处理ML307R的MQTT URC。**

```
MQTT回调链表（支持多模块订阅）:
  at_mqtt_register_callback(cb)    注册回调（最多4个槽位）
  at_mqtt_unregister_callback(cb)  注销回调
  收到+MQTTURC: "message",...时，分发给所有已注册回调

关键MQTT命令:
  at_mqtt_config(host, port, ...)  → AT+MQTTCONN=<id>,"<host>",<port>,...
  at_mqtt_connect()                 → AT+MQTTCONN=<id>
  at_mqtt_subscribe(topic, qos)     → AT+MQTTSUB=<id>,"<topic>",<qos>
  at_mqtt_publish(topic, qos, data) → AT+MQTTPUB=<id>,"<topic>",<qos>,0,0,<len>,"<data>"
  at_mqtt_disconnect()              → AT+MQTTDISC=<id>
```

### 5.3 `ml307r_init.c` — ML307R初始化状态机

**职责：按顺序初始化ML307R模块，确保网络就绪。**

```
状态流程:
  INIT → SIM_CHECK → REGISTERED → DIAL → CONNECTED
                                        ↓
                                      ERROR（任意步骤失败）

初始化序列:
  1. AT         → AT通信测试
  2. ATE0       → 关闭回显
  3. AT+CPIN?   → 检查SIM卡（等待"READY"）
  4. AT+CEREG?  → 等待网络注册（最多30次×1秒，等待"+CEREG: 0,1"或"1,1"）
  5. AT+MIPCALL=1,1 → 激活PDP数据连接
```

### 5.4 `task_cloud.c` — 云端通信主任务

**职责：协调ML307R初始化 → MQTT连接 → 上报数据。**

```
Cloud_Task 执行顺序:
  阶段1: ml307r_init()           初始化4G模块（失败则每5秒重试）
  阶段2: mqtt_client_connect()   连接MQTT（失败则每3秒重试）
  阶段3: ota_register_mqtt_callback() 注册OTA回调
  阶段4: osEventFlagsSet()        通知OTA_Task可以开始（Bug4修复）
  阶段5: 主循环：每5秒上报 {"status":"online"}
```

### 5.5 `task_ota.c` — OTA固件升级

**职责：接收云端OTA命令，通过XMODEM协议接收固件。**

```
OTA_Task 启动时:
  osEventFlagsWait(CLOUD_READY_FLAG) → 等待Cloud_Task完成初始化

OTA状态机:
  IDLE → WAIT_READY → RECEIVING → COMPLETE → NVIC_SystemReset()

MQTT命令解析:
  {"cmd":"ota_start"}       → 进入WAIT_READY
  {"cmd":"update"}          → 同上
  update_fw <ver> <len>     → 兼容旧格式

收到ota_start后:
  uart_at_set_mode(UART_AT_MODE_XMODEM, xmodem_process_byte)
  xmodem_init() + xmodem_start()
  UART字节直接路由到XMODEM接收器
```

### 5.6 `xmodem.c` — XMODEM-CRC16接收协议

**职责：实现XMODEM-CRC16协议，接收固件数据块。**

```
XMODEM数据包结构（128字节块）:
  SOH(1) | Block#(1) | ~Block#(1) | Data(128) | CRC16_H(1) | CRC16_L(1)
  共133字节

状态机:
  IDLE        → 等待NAK或'C'启动
  RECEIVING   → 接收数据块，校验CRC，回复ACK/NAK
  COMPLETE    → 收到EOT(0x04)或块号归零
  ERROR       → 收到CAN(0x18)或过多错误

每接收一个块:
  CRC校验成功 → 写入SPI Flash → at_send_raw(ACK)
  CRC校验失败 → at_send_raw(NAK) → 等待重发

参考ESP32实现: wifi-ge-esp32-c3-master/main/mcu.c
  块号255后归零处理
  支持EOT(0x04)结束
  支持CAN(0x18)取消
```

### 5.7 `flash_spi.c` — 外部SPI Flash驱动

**职责：W25Qxx系列Flash读写，用于OTA固件暂存。**

```
操作:
  flash_spi_write(addr, data, len)  页编程（自动写使能）
  flash_spi_read(addr, data, len)   标准读取
  flash_spi_erase(addr, len)        扇区擦除（4KB，400ms延迟）

OTA_FLASH_ADDR = 0x00000000
```

### 5.8 `config.h` — 设备配置

```c
DEVICE_SN         "30AEA4C75B61"
DEVICE_PRODUCT_MODEL "dm-mq100"
MCU_FIRMWARE_VERSION "1.0.0"
MQTT_SERVER       "mqtt.cn.dream-maker.com"
MQTT_PORT         8883 (TLS)
CMD_TOPIC         "$sys/cn/dream-maker.com/cmd"
DATA_TOPIC        "$sys/cn/dream-maker.com/data"
```

## 6. 数据流

### 6.1 云端下发命令（MQTT → STM32）

```
Dream Maker Cloud
  → MQTT publish to "$sys/cn/dream-maker.com/cmd"
  → ML307R收到 → 通过UART2发送 +MQTTURC: "message",...
  → uart_at.c HAL_UARTEx_RxEventCallback (IDLE中断)
  → parse_rx_lines() 解析行
  → mqtt_urc_handler() (at_parser.c)
  → 遍历MQTT回调链表，分发给:
      - on_cloud_mqtt_message (task_ota.c)
  → OTA_Task收到 {"cmd":"ota_start"} → 进入OTA流程
```

### 6.2 设备上报数据（STM32 → 云端）

```
Cloud_Task 主循环
  → mqtt_client_publish(DATA_TOPIC, payload)
  → at_mqtt_publish(topic, 0, payload)  (at_parser.c)
  → at_send_command("AT+MQTTPUB=0,...") (uart_at.c)
  → UART2 TX → ML307R → MQTT Broker → Cloud
```

### 6.3 OTA固件升级

```
Cloud → MQTT "$sys/cn/dream-maker.com/cmd" {"cmd":"ota_start"}
  → on_cloud_mqtt_message() 收到命令
  → ota_state = WAIT_READY

Cloud_Task 检测到 ota_state==WAIT_READY
  → ota_start_update()
  → uart_at_set_mode(XMODEM) 切换UART模式
  → xmodem_start()
  → ML307R开始XMODEM发送（每块128字节+CRC16）

每收到一帧XMODEM数据:
  xmodem_process_byte(byte) (在ISR中被调用)
  → 凑够133字节 → CRC校验 → flash_spi_write() → at_send_raw(ACK)

传输完成后:
  xmodem_get_state() == XMODEM_COMPLETE
  → uart_at_set_mode(AT) 切换回AT模式
  → NVIC_SystemReset() 复位，bootloader加载新固件
```

## 7. 任务间同步机制

```
app_main.c:
  g_cloud_ready_flag = osEventFlagsNew()

Cloud_Task (完成后设置):
  osEventFlagsSet(g_cloud_ready_flag, CLOUD_READY_FLAG_ID)

OTA_Task (启动时等待):
  osEventFlagsWait(g_cloud_ready_flag, CLOUD_READY_FLAG_ID,
                   osFlagsWaitAll | osFlagsNoClear, osWaitForever)
```

## 8. 移植/适配要点

### 8.1 更换IoT平台

只需修改 `config.h` 中的：
- `MQTT_SERVER` / `MQTT_PORT`
- `MQTT_CLIENT_ID` / `MQTT_USERNAME` / `MQTT_PASSWORD`
- `CMD_TOPIC` / `DATA_TOPIC`
- 对应修改 `at_parser.c` 中的MQTT AT命令格式

### 8.2 启用LVGL

在 `task_ui.c` 中：
- `#if 0` → `#if 1` 解除LVGL代码注释
- 实现 `disp_flush()` 等显示驱动
- 替换 `osDelay(10)` 为 `lv_task_handler()`

### 8.3 更换4G模块

主要修改 `ml307r_init.c` 中的AT指令序列（网络注册/PDP激活命令可能不同），以及 `at_parser.c` 中的MQTT命令格式。

### 8.4 更换Flash芯片

修改 `flash_spi.c` 中的命令字和时序（W25Qxx系列基本兼容）。

## 9. 调试技巧

```c
// 调试打印（通过UART1）
uart_debug_printf("ML307R state: %d\r\n", ml307r_get_state());

// 临时添加AT命令测试（在Cloud_Task中）
char resp[256];
at_send_command("AT+CSQ", "OK", 3000, resp, sizeof(resp));
uart_debug_printf("CSQ: %s\r\n", resp);

// XMODEM调试（在xmodem_process_byte中）
uart_debug_printf("XMODEM byte: 0x%02X\r\n", byte);
```
