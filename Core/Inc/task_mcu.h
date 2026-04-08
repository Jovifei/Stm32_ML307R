#ifndef TASK_MCU_H
#define TASK_MCU_H

#include <stdint.h>
#include <stdbool.h>

// ==================== 属性值类型 ====================
typedef enum {
    MCU_VALUE_TYPE_FALSE = 0,
    MCU_VALUE_TYPE_TRUE,
    MCU_VALUE_TYPE_INT,
    MCU_VALUE_TYPE_FLOAT,
    MCU_VALUE_TYPE_STRING,
    MCU_VALUE_TYPE_NOT_FOUND = -4003,
    MCU_VALUE_TYPE_INVALID_PARAM = -4005,
} mcu_value_type_t;

// ==================== 属性参数结构 ====================
typedef struct {
    int siid;                       // 服务ID
    int piid;                       // 属性ID
    uint8_t flag;                   // 是否有效
    mcu_value_type_t value_type;    // 值类型
    int32_t value;                  // 整数值
    float f_value;                  // 浮点值
    char *s_value;                  // 字符串值
} mcu_msg_params_t;

// ==================== 属性上报标志位 ====================
typedef enum {
    MCU_PROP_DEVICE_INFO   = 0x00000001,  // 设备信息
    MCU_PROP_RUN_STATE     = 0x00000002,  // 运行状态
    MCU_PROP_POWER         = 0x00000004,  // 功率
    MCU_PROP_ENERGY        = 0x00000008,  // 能量
    MCU_PROP_TEMPERATURE   = 0x00000010,  // 温度
    MCU_PROP_VOLTAGE       = 0x00000020,  // 电压
    MCU_PROP_FAULT         = 0x00000040,  // 故障
    // 可扩展更多属性...
} mcu_prop_flag_t;

// ==================== 设备信息结构 ====================
typedef struct {
    char sn[32];
    char model[32];
    char sw_version[16];
    char hw_version[16];
    int run_state;
    float power;
    float today_energy;
    float lifetime_energy;
    float temperature;
    float voltage;
    uint32_t fault_code;
} mcu_device_info_t;

// ==================== MCU任务API ====================

// 初始化MCU任务
void MCU_Task_Init(void);

// MCU任务主体
void MCU_Task(void *argument);

// 设置属性变化标志（由应用层调用，触发属性上报）
void mcu_set_prop_changed(mcu_prop_flag_t prop);

// 获取设备信息（供MQTT上报使用）
const mcu_device_info_t* mcu_get_device_info(void);

// 更新设备信息（由应用层调用）
void mcu_update_device_info(const mcu_device_info_t *info);

// 处理云端MQTT消息（由at_parser的MQTT回调调用）
// 解析get_properties/set_properties/action并返回响应payload
// 返回的payload需要通过MQTT发布
int mcu_handle_cloud_message(const char *topic, const char *payload, int len,
                             char *response, int resp_max_len);

#endif // TASK_MCU_H
