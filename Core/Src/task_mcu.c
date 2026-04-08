#include "task_mcu.h"
#include "mqtt_client.h"
#include "at_parser.h"
#include "app_main.h"
#include "config.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ==================== 常量 ====================
#define PROP_REPORT_INTERVAL_MS  30000   // 属性定期上报间隔(30秒)
#define DEVICE_INFO_SIID         1       // 设备信息服务ID
#define SYSTEM_SIID             2        // 系统服务ID

// ==================== 静态变量 ====================
static mcu_device_info_t s_device_info;
static uint32_t s_prop_flags = 0;           // 待上报属性标志
static uint32_t s_prop_timer_count = 0;     // 属性上报计时器
static osMutexId_t s_mutex = NULL;

// ==================== 内部函数 ====================

// 解析属性参数字符串（空格分隔）
// 格式: <siid> <piid> [<siid> <piid>]...
static void parse_property_params(char *string, mcu_msg_params_t *params_list, int max_count) {
    char *root[64];
    int num = 0;

    memset(params_list, 0, sizeof(mcu_msg_params_t) * max_count);

    root[0] = string;
    for (num = 1; num < 64; num++) {
        root[num] = strstr(root[num - 1], " ");
        if (root[num]) {
            *root[num] = '\0';
            root[num]++;
        } else {
            break;
        }
    }

    // 解析 siid piid 对
    int index = 0;
    for (int i = 0; i < num && index < max_count; i += 2) {
        if (i + 1 >= num) break;

        // 检查是否为数字
        if (*root[i] >= '0' && *root[i] <= '9') {
            sscanf(root[i], "%d", &params_list[index].siid);
            sscanf(root[i + 1], "%d", &params_list[index].piid);
            params_list[index].flag = 1;
            index++;
        } else {
            break;
        }
    }
}

// 解析设置属性参数字符串
// 格式: <siid> <piid> <value> <siid> <piid> <value>...
static void parse_set_params(char *string, mcu_msg_params_t *params_list, int max_count) {
    char *root[64];
    int num = 0;

    memset(params_list, 0, sizeof(mcu_msg_params_t) * max_count);

    root[0] = string;
    for (num = 1; num < 64; num++) {
        root[num] = strstr(root[num - 1], " ");
        if (root[num]) {
            *root[num] = '\0';
            root[num]++;
        } else {
            break;
        }
    }

    // 解析 siid piid value 三元组
    int index = 0;
    for (int i = 0; i < num && index < max_count; i += 3) {
        if (i + 2 >= num) break;

        if (*root[i] >= '0' && *root[i] <= '9') {
            sscanf(root[i], "%d", &params_list[index].siid);
            sscanf(root[i + 1], "%d", &params_list[index].piid);

            // 解析value
            char *val = root[i + 2];
            if (strcmp(val, "true") == 0) {
                params_list[index].value = 1;
                params_list[index].value_type = MCU_VALUE_TYPE_TRUE;
            } else if (strcmp(val, "false") == 0) {
                params_list[index].value = 0;
                params_list[index].value_type = MCU_VALUE_TYPE_FALSE;
            } else if (*val == '"') {
                // 字符串类型
                val++;
                char *end = val + strlen(val) - 1;
                if (*end == '"') *end = '\0';
                params_list[index].s_value = val;
                params_list[index].value_type = MCU_VALUE_TYPE_STRING;
            } else if ((*val >= '0' && *val <= '9') || *val == '-') {
                // 数字类型
                if (strchr(val, '.') != NULL) {
                    sscanf(val, "%f", &params_list[index].f_value);
                    params_list[index].value_type = MCU_VALUE_TYPE_FLOAT;
                } else {
                    sscanf(val, "%ld", &params_list[index].value);
                    params_list[index].value_type = MCU_VALUE_TYPE_INT;
                }
            }
            params_list[index].flag = 1;
            index++;
        } else {
            break;
        }
    }
}

// 处理get_properties命令
// 返回响应字符串到response buffer
static void handle_get_properties(const char *params, char *response, int resp_max_len) {
    mcu_msg_params_t params_list[24];
    char params_copy[256];

    strncpy(params_copy, params, sizeof(params_copy) - 1);
    params_copy[sizeof(params_copy) - 1] = '\0';
    parse_property_params(params_copy, params_list, 24);

    int offset = 0;
    offset += snprintf(response + offset, resp_max_len - offset, "result");

    for (int i = 0; i < 24 && params_list[i].flag; i++) {
        int siid = params_list[i].siid;
        int piid = params_list[i].piid;

        if (siid == DEVICE_INFO_SIID) {
            // 设备信息服务 (SIID=1)
            switch (piid) {
                case 1: // 产品型号
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 1 1 0 \"%s\"", DEVICE_PRODUCT_MODEL);
                    break;
                case 2: // 产品SN
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 1 2 0 \"%s\"", DEVICE_SN);
                    break;
                case 3: // 固件版本
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 1 3 0 \"%s\"", MCU_FIRMWARE_VERSION);
                    break;
                case 4: // 硬件版本
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 1 4 0 \"%s\"", "V1.0.0");
                    break;
                default:
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 1 %d -4003", piid);  // 属性不存在
                    break;
            }
        } else if (siid == SYSTEM_SIID) {
            // 系统服务 (SIID=2)
            osMutexAcquire(s_mutex, osWaitForever);
            switch (piid) {
                case 1: // 运行状态
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 2 1 0 %d", s_device_info.run_state);
                    break;
                case 2: // 当前功率
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 2 2 0 %.2f", s_device_info.power);
                    break;
                case 3: // 今日发电量
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 2 3 0 %.2f", s_device_info.today_energy);
                    break;
                case 4: // 累计发电量
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 2 4 0 %.2f", s_device_info.lifetime_energy);
                    break;
                case 5: // 温度
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 2 5 0 %.1f", s_device_info.temperature);
                    break;
                case 6: // 电网电压
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 2 6 0 %.1f", s_device_info.voltage);
                    break;
                case 7: // 故障代码
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 2 7 0 %u", s_device_info.fault_code);
                    break;
                default:
                    offset += snprintf(response + offset, resp_max_len - offset,
                                     " 2 %d -4003", piid);
                    break;
            }
            osMutexRelease(s_mutex);
        } else {
            offset += snprintf(response + offset, resp_max_len - offset,
                             " %d %d -4003", siid, piid);
        }
    }
}

// 处理set_properties命令
// 返回响应字符串到response buffer
static void handle_set_properties(const char *params, char *response, int resp_max_len) {
    mcu_msg_params_t params_list[16];
    char params_copy[256];

    strncpy(params_copy, params, sizeof(params_copy) - 1);
    params_copy[sizeof(params_copy) - 1] = '\0';
    parse_set_params(params_copy, params_list, 16);

    int offset = 0;
    offset += snprintf(response + offset, resp_max_len - offset, "result");

    for (int i = 0; i < 16 && params_list[i].flag; i++) {
        int siid = params_list[i].siid;
        int piid = params_list[i].piid;
        int result_code = 0;  // 0 = 成功

        // 处理可写属性
        if (siid == DEVICE_INFO_SIID && piid == 4) {
            // 硬件版本写入（示例）
            // 可以添加存储逻辑
        }
        // else if (siid == SYSTEM_SIID && piid == xxx) { ... }

        offset += snprintf(response + offset, resp_max_len - offset,
                          " %d %d %d", siid, piid, result_code);
    }
}

// 处理action命令
static void handle_action(const char *params, char *response, int resp_max_len) {
    int siid = 0, aiid = 0;
    char action_params[128] = "";

    sscanf(params, "%d %d %s", &siid, &aiid, action_params);

    int offset = snprintf(response, resp_max_len, "result");

    // 根据siid/aiid处理不同动作
    if (siid == SYSTEM_SIID && aiid == 1) {
        // 重启设备动作
        offset += snprintf(response + offset, resp_max_len - offset, " 2 1 0");
        // 可以添加实际重启逻辑
    } else {
        offset += snprintf(response + offset, resp_max_len - offset, " %d %d -4003", siid, aiid);
    }
}

// 构建属性变化上报payload
static int build_properties_changed(char *payload, int max_len) {
    int offset = 0;

    osMutexAcquire(s_mutex, osWaitForever);

    // 检查需要上报的属性
    if (s_prop_flags & MCU_PROP_RUN_STATE) {
        offset += snprintf(payload + offset, max_len - offset,
                          "properties_changed 2 1 %d ", s_device_info.run_state);
    }
    if (s_prop_flags & MCU_PROP_POWER) {
        offset += snprintf(payload + offset, max_len - offset,
                          "properties_changed 2 2 %.2f ", s_device_info.power);
    }
    if (s_prop_flags & MCU_PROP_ENERGY) {
        offset += snprintf(payload + offset, max_len - offset,
                          "properties_changed 2 3 %.2f 2 4 %.2f ",
                          s_device_info.today_energy, s_device_info.lifetime_energy);
    }
    if (s_prop_flags & MCU_PROP_TEMPERATURE) {
        offset += snprintf(payload + offset, max_len - offset,
                          "properties_changed 2 5 %.1f ", s_device_info.temperature);
    }
    if (s_prop_flags & MCU_PROP_VOLTAGE) {
        offset += snprintf(payload + offset, max_len - offset,
                          "properties_changed 2 6 %.1f ", s_device_info.voltage);
    }
    if (s_prop_flags & MCU_PROP_FAULT) {
        offset += snprintf(payload + offset, max_len - offset,
                          "properties_changed 2 7 %u ", s_device_info.fault_code);
    }

    osMutexRelease(s_mutex);

    return offset;
}

// ==================== 公开API ====================

// MQTT消息回调处理函数（静态，在init时注册）
static void mcu_mqtt_callback(const char *topic, const char *payload, int len) {
    char response[512];
    int ret = mcu_handle_cloud_message(topic, payload, len, response, sizeof(response));
    if (ret == 0 && response[0] != '\0') {
        // 将响应发回云端
        mqtt_client_publish(DATA_TOPIC, response, 0);
    }
}

void MCU_Task_Init(void) {
    if (s_mutex == NULL) {
        osMutexAttr_t mutex_attr = { .name = "mcu_info" };
        s_mutex = osMutexNew(&mutex_attr);
    }

    // 注册MQTT消息回调（处理云端下行的get/set_properties和action）
    at_mqtt_register_callback(mcu_mqtt_callback);

    // 初始化默认设备信息
    memset(&s_device_info, 0, sizeof(s_device_info));
    strncpy(s_device_info.model, DEVICE_PRODUCT_MODEL, sizeof(s_device_info.model) - 1);
    strncpy(s_device_info.sw_version, MCU_FIRMWARE_VERSION, sizeof(s_device_info.sw_version) - 1);
    strncpy(s_device_info.sn, DEVICE_SN, sizeof(s_device_info.sn) - 1);
    s_device_info.run_state = 0;
    s_device_info.power = 0.0f;
    s_device_info.today_energy = 0.0f;
    s_device_info.lifetime_energy = 0.0f;
    s_device_info.temperature = 25.0f;
    s_device_info.voltage = 220.0f;
    s_device_info.fault_code = 0;
}

void MCU_Task(void *argument) {
    (void)argument;

    MCU_Task_Init();

    // 等待Cloud_Task初始化完成（MQTT连接完成）
    osEventFlagsId_t flag = task_sync_get_cloud_ready_flag();
    if (flag != NULL) {
        osEventFlagsWait(flag, CLOUD_READY_FLAG_ID,
                         osFlagsWaitAll | osFlagsNoClear,
                         osWaitForever);
    }

    // 初始化完成后首次上报设备信息
    mcu_set_prop_changed(MCU_PROP_DEVICE_INFO | MCU_PROP_RUN_STATE);

    while (1) {
        osDelay(100);  // 100ms周期
        s_prop_timer_count += 100;

        // 属性上报（每30秒一次，或者有变化时立即上报）
        if (s_prop_flags != 0) {
            char payload[512];
            int len = build_properties_changed(payload, sizeof(payload));
            if (len > 0) {
                mqtt_client_publish(DATA_TOPIC, payload, 0);
                s_prop_flags = 0;  // 清除已上报的标志
            }
        } else if (s_prop_timer_count >= PROP_REPORT_INTERVAL_MS) {
            // 定期上报（无变化时也上报一次保持连接）
            s_prop_timer_count = 0;
            char payload[128];
            osMutexAcquire(s_mutex, osWaitForever);
            snprintf(payload, sizeof(payload),
                    "properties_changed 2 1 %d 2 2 %.2f 2 3 %.2f 2 4 %.2f",
                    s_device_info.run_state,
                    s_device_info.power,
                    s_device_info.today_energy,
                    s_device_info.lifetime_energy);
            osMutexRelease(s_mutex);
            mqtt_client_publish(DATA_TOPIC, payload, 0);
        }
    }
}

void mcu_set_prop_changed(mcu_prop_flag_t prop) {
    osMutexAcquire(s_mutex, osWaitForever);
    s_prop_flags |= (uint32_t)prop;
    osMutexRelease(s_mutex);
}

const mcu_device_info_t* mcu_get_device_info(void) {
    return &s_device_info;
}

void mcu_update_device_info(const mcu_device_info_t *info) {
    if (info == NULL) return;

    osMutexAcquire(s_mutex, osWaitForever);

    int changed = 0;
    if (s_device_info.run_state != info->run_state) {
        s_device_info.run_state = info->run_state;
        changed |= MCU_PROP_RUN_STATE;
    }
    if (s_device_info.power != info->power) {
        s_device_info.power = info->power;
        changed |= MCU_PROP_POWER;
    }
    if (s_device_info.today_energy != info->today_energy) {
        s_device_info.today_energy = info->today_energy;
        changed |= MCU_PROP_ENERGY;
    }
    if (s_device_info.lifetime_energy != info->lifetime_energy) {
        s_device_info.lifetime_energy = info->lifetime_energy;
        changed |= MCU_PROP_ENERGY;
    }
    if (s_device_info.temperature != info->temperature) {
        s_device_info.temperature = info->temperature;
        changed |= MCU_PROP_TEMPERATURE;
    }
    if (s_device_info.voltage != info->voltage) {
        s_device_info.voltage = info->voltage;
        changed |= MCU_PROP_VOLTAGE;
    }
    if (s_device_info.fault_code != info->fault_code) {
        s_device_info.fault_code = info->fault_code;
        changed |= MCU_PROP_FAULT;
    }

    s_prop_flags |= changed;
    osMutexRelease(s_mutex);
}

int mcu_handle_cloud_message(const char *topic, const char *payload, int len,
                             char *response, int resp_max_len) {
    (void)topic;

    char buf[512];
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
    strncpy(buf, payload, len);
    buf[len] = '\0';

    // 查找命令类型
    char *cmd_start = strstr(buf, "down ");
    if (cmd_start == NULL) {
        // 尝试JSON格式 {"cmd":"get_properties",...}
        char cmd[32] = {0};
        if (sscanf(buf, "{\"cmd\":\"%31[^\"]\"", cmd) >= 1) {
            if (strcmp(cmd, "get_properties") == 0) {
                // 提取params字段
                char *params = strstr(buf, "\"params\":\"");
                if (params) {
                    params += 9;
                    char *params_end = strchr(params, '"');
                    if (params_end) *params_end = '\0';
                    handle_get_properties(params, response, resp_max_len);
                    return 0;
                }
            } else if (strcmp(cmd, "set_properties") == 0) {
                char *params = strstr(buf, "\"params\":\"");
                if (params) {
                    params += 9;
                    char *params_end = strchr(params, '"');
                    if (params_end) *params_end = '\0';
                    handle_set_properties(params, response, resp_max_len);
                    return 0;
                }
            } else if (strcmp(cmd, "action") == 0) {
                char *params = strstr(buf, "\"params\":\"");
                if (params) {
                    params += 9;
                    char *params_end = strchr(params, '"');
                    if (params_end) *params_end = '\0';
                    handle_action(params, response, resp_max_len);
                    return 0;
                }
            }
        }
        return -1;
    }

    // 文本格式: down get_properties/set_properties/action [params]
    cmd_start += 5;  // 跳过 "down "

    if (strncmp(cmd_start, "get_properties", 14) == 0) {
        char *params = cmd_start + 14;
        while (*params == ' ') params++;
        handle_get_properties(params, response, resp_max_len);
    } else if (strncmp(cmd_start, "set_properties", 14) == 0) {
        char *params = cmd_start + 14;
        while (*params == ' ') params++;
        handle_set_properties(params, response, resp_max_len);
    } else if (strncmp(cmd_start, "action", 6) == 0) {
        char *params = cmd_start + 6;
        while (*params == ' ') params++;
        handle_action(params, response, resp_max_len);
    } else {
        return -1;
    }

    return 0;
}
