#include "mqtt_client.h"
#include "at_parser.h"
#include "at_config.h"
#include "config.h"
#include "cmsis_os2.h"
#include <string.h>

static mqtt_state_t s_state = MQTT_STATE_DISCONNECTED;
static osMutexId_t s_mutex = NULL;

/*---------------------------------------------------------------------------
 Name        : mqtt_client_connect
 Input       : None
 Output      : int
 Description :
 建立设备到云端的 MQTT 连接（基于 ML307R AT 指令栈），并完成必要的预配置与订阅。

 连接流程（按当前实现分阶段）：
 - 阶段1：预配置 + TLS 初始化
   - 调用 `at_provision_and_tls_init()`：
     - 优先从外部 SPI Flash 读取已存储的 device_id/device_key 与证书状态；
     - 若无有效凭证，则通过 HTTPS 向服务器请求 device_id/device_key，并获取证书（PEM）保存到 Flash；
     - 证书写入 ML307R 模组的动作在当前工程中由 `at_ssl_init_full()` 提供，但此处流程以
       at_parser 的封装为准。
 - 阶段2：配置 MQTT 连接参数（启用 SSL/TLS）
   - `at_mqtt_config(MQTT_SERVER, MQTT_PORT, DEVICE_SN, g_device_id, g_device_key)`
 - 阶段3：建立 MQTT 连接
   - `at_mqtt_connect()`
 - 阶段4：订阅命令主题（下行）
   - `at_mqtt_subscribe(CMD_TOPIC, 1)`

 状态机与线程安全：
 - 使用 `s_mutex` 保护 `s_state` 的更新，避免多任务并发调用造成状态混乱；
 - 状态流转：DISCONNECTED -> CONNECTING -> CONNECTED；任一步失败进入 ERROR。

 返回值约定：
 - 返回 0：连接与订阅成功，`s_state=MQTT_STATE_CONNECTED`
 - 返回 -1：任一步失败，`s_state=MQTT_STATE_ERROR`
---------------------------------------------------------------------------*/
int mqtt_client_connect(void)
{
    if (s_mutex == NULL)
    {
        s_mutex = osMutexNew(NULL);
    }

    osMutexAcquire(s_mutex, osWaitForever);
    s_state = MQTT_STATE_CONNECTING;

    // 阶段1: 预配置+TLS初始化
    // 从Flash加载凭证或从服务器获取+写入证书到模块
    int ret = at_provision_and_tls_init();
    if (ret != 0)
    {
        s_state = MQTT_STATE_ERROR;
        osMutexRelease(s_mutex);
        return -1;
    }

    // 阶段2: 配置MQTT连接参数（使用SSL/TLS）
    ret = at_mqtt_config(MQTT_SERVER, MQTT_PORT, DEVICE_SN, g_device_id, g_device_key);
    if (ret != 0)
    {
        s_state = MQTT_STATE_ERROR;
        osMutexRelease(s_mutex);
        return -1;
    }

    // 阶段3: 建立MQTT连接
    ret = at_mqtt_connect();
    if (ret != 0)
    {
        s_state = MQTT_STATE_ERROR;
        osMutexRelease(s_mutex);
        return -1;
    }

    // 阶段4: 订阅命令主题
    ret = at_mqtt_subscribe(CMD_TOPIC, 1);
    if (ret != 0)
    {
        s_state = MQTT_STATE_ERROR;
        osMutexRelease(s_mutex);
        return -1;
    }

    s_state = MQTT_STATE_CONNECTED;
    osMutexRelease(s_mutex);
    return 0;
}

/*---------------------------------------------------------------------------
 Name        : mqtt_client_disconnect
 Input       : None
 Output      : int
 Description :
 主动断开 MQTT 连接，并将内部状态机切换为 DISCONNECTED。

 行为说明：
 - 若互斥锁尚未创建（通常表示从未成功初始化/连接），直接返回失败；
 - 否则在互斥保护下调用 `at_mqtt_disconnect()` 断开 ML307R 的 MQTT 连接；
 - 不论 AT 返回结果如何，都会将 `s_state` 置为 DISCONNECTED，表示上层应重新建链。

 返回值约定：
 - 返回 0：AT 断开命令返回成功
 - 返回 -1：互斥未初始化或 AT 断开失败
---------------------------------------------------------------------------*/
int mqtt_client_disconnect(void)
{
    if (s_mutex == NULL)
        return -1;
    osMutexAcquire(s_mutex, osWaitForever);
    int ret = at_mqtt_disconnect();
    s_state = MQTT_STATE_DISCONNECTED;
    osMutexRelease(s_mutex);
    return (ret == 0) ? 0 : -1;
}

/*---------------------------------------------------------------------------
 Name        : mqtt_client_publish
 Input       : const char *topic, const char *payload, int qos
 Output      : int
 Description :
 发布一条 MQTT 消息到指定主题（透传到 ML307R 的 `AT+MQTTPUB` 封装）。

 输入参数：
 - topic：发布主题字符串
 - payload：消息内容（文本/JSON 等）
 - qos：QoS 等级（0/1/2），具体支持取决于模组 AT 命令实现

 返回值约定：
 - 返回 0：发布成功（底层 AT 返回 OK）
 - 返回非 0：发布失败或参数错误（由 `at_mqtt_publish` 返回）

 注意：
 - 当前实现不检查 `s_state` 是否已连接；调用方若需要严格控制，应先检查
   `mqtt_client_get_state()==MQTT_STATE_CONNECTED`。
---------------------------------------------------------------------------*/
int mqtt_client_publish(const char *topic, const char *payload, int qos)
{
    return at_mqtt_publish(topic, qos, payload, 0);
}

/*---------------------------------------------------------------------------
 Name        : mqtt_client_get_state
 Input       : None
 Output      : mqtt_state_t
 Description :
 获取 MQTT 客户端当前状态机状态。

 线程安全：
 - 若互斥锁未创建，直接返回当前静态变量 `s_state`；
 - 否则在互斥保护下读取，保证读取一致。

 使用场景：
 - 上层任务决定是否需要重连/退避；
 - UI/日志模块显示当前连接状态；
 - 发布前快速判断是否处于 CONNECTED。
---------------------------------------------------------------------------*/
mqtt_state_t mqtt_client_get_state(void)
{
    if (s_mutex == NULL)
        return s_state;
    osMutexAcquire(s_mutex, osWaitForever);
    mqtt_state_t st = s_state;
    osMutexRelease(s_mutex);
    return st;
}
