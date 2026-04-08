#include "mqtt_client.h"
#include "at_parser.h"
#include "at_config.h"
#include "config.h"
#include "cmsis_os2.h"
#include <string.h>

static mqtt_state_t s_state = MQTT_STATE_DISCONNECTED;
static osMutexId_t s_mutex = NULL;

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

int mqtt_client_publish(const char *topic, const char *payload, int qos)
{
    return at_mqtt_publish(topic, qos, payload, 0);
}

mqtt_state_t mqtt_client_get_state(void)
{
    if (s_mutex == NULL)
        return s_state;
    osMutexAcquire(s_mutex, osWaitForever);
    mqtt_state_t st = s_state;
    osMutexRelease(s_mutex);
    return st;
}
