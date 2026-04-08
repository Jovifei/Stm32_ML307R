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
/*---------------------------------------------------------------------------
 Name        : parse_property_params
 Input       : char *string, mcu_msg_params_t *params_list, int max_count
 Output      : None
 Description :
 解析云端下行命令中属性查询参数列表（get_properties），将参数转换为结构化的
 `mcu_msg_params_t` 数组，供后续属性读取与响应构建使用。

 输入参数：
 - string：可修改的参数字符串缓冲区。函数会在分割 token 时写入 '\0'，
   因此必须是可写内存（不可传入只读常量）。
 - params_list：输出数组指针，元素类型为 `mcu_msg_params_t`；函数会清零并填充。
 - max_count：params_list 最大可填充条目数，防止越界写入。

 支持的参数格式：
 - 空格分隔的成对数字：`<siid> <piid> <siid> <piid> ...`
   例如：`"2 1 2 2 1 3"` 表示读取多个服务/属性。

 解析策略：
 - 先在原字符串上就地切分，得到 token 指针数组 root[]；
 - 再以 2 个 token 为一组（siid/piid）进行解析；
 - 当遇到非数字 token 或 token 不成对时停止解析；
 - 对每个成功解析的条目设置 `flag=1` 表示有效。

 典型使用场景：
 - `handle_get_properties()` 在解析云端参数后按 (siid,piid) 逐项读取设备属性。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : parse_set_params
 Input       : char *string, mcu_msg_params_t *params_list, int max_count
 Output      : None
 Description :
 解析云端下行命令中属性设置参数列表（set_properties），将参数转换为结构化的
 `mcu_msg_params_t` 数组，供后续写属性逻辑与响应构建使用。

 输入参数：
 - string：可修改的参数字符串缓冲区，函数会在切分 token 时写入 '\0'。
 - params_list：输出数组指针；函数会清零并逐项填充。
 - max_count：最多解析/填充的条目数，避免越界。

 支持的参数格式（空格分隔的三元组）：
 - `<siid> <piid> <value> <siid> <piid> <value> ...`
 value 支持以下类型（按当前实现）：
 - true/false：布尔类型，对应 `MCU_VALUE_TYPE_TRUE/FALSE`
 - "xxx"：双引号包裹的字符串（会去掉首尾引号），对应 `MCU_VALUE_TYPE_STRING`
 - 数字：整数或浮点（含 '.' 判定为浮点），对应 `MCU_VALUE_TYPE_INT/FLOAT`

 输出结构字段约定：
 - siid/piid：属性标识
 - value/value_type：对整型/布尔值使用 value + value_type
 - f_value/value_type：对浮点使用 f_value + value_type
 - s_value/value_type：对字符串使用 s_value（指向 string 内部切分后的区域）
 - flag=1：表示该条目有效

 注意事项：
 - 字符串类型的 s_value 指针依赖入参 string 的生命周期，调用者需保证
   在后续处理期间该缓冲区仍然有效。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : handle_get_properties
 Input       : const char *params, char *response, int resp_max_len
 Output      : None
 Description :
 处理云端下行的 get_properties 请求：解析参数列表并把读取结果按协议格式拼接到
 response 缓冲区中。

 输入参数：
 - params：属性参数字符串，格式为 `<siid> <piid> ...`（空格分隔）。
 - response：输出响应缓冲区指针，用于返回组装后的文本响应。
 - resp_max_len：response 最大长度，函数内部会尽量避免写越界。

 输出协议（当前实现的文本格式）：
 - 以 "result" 开头，随后每个属性追加：
   - 成功：` <siid> <piid> 0 <value>`
   - 失败：` <siid> <piid> -4003`（属性不存在）

 属性映射（按当前实现）：
 - DEVICE_INFO_SIID(1)：产品型号/序列号/固件版本/硬件版本等静态信息
 - SYSTEM_SIID(2)：运行状态/功率/电量/温度/电压/故障码等动态信息（需互斥读）

 线程安全：
 - 读取 SYSTEM_SIID 对应的 `s_device_info` 时会加 `s_mutex`，保证读取一致。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : handle_set_properties
 Input       : const char *params, char *response, int resp_max_len
 Output      : None
 Description :
 处理云端下行的 set_properties 请求：解析要写入的属性列表，并按协议格式构建
 每个属性写入的结果码到 response 缓冲区中。

 输入参数：
 - params：设置参数字符串，格式为 `<siid> <piid> <value> ...`
 - response：输出响应缓冲区
 - resp_max_len：输出缓冲区最大长度

 输出协议（当前实现）：
 - 以 "result" 开头，随后每个属性追加：
   ` <siid> <piid> <result_code>`
   其中 result_code 当前固定为 0（成功），用于示例占位。

 业务说明：
 - 当前实现仅完成参数解析与响应框架，实际“写属性并持久化”的逻辑尚未补齐；
 - 需要扩展时，建议按 (siid,piid) 分发到具体模块，并根据写入结果返回非 0 错误码。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : handle_action
 Input       : const char *params, char *response, int resp_max_len
 Output      : None
 Description :
 处理云端下行的 action 请求：解析 (siid, aiid, action_params)，并根据动作类型
 构建对应的执行结果响应。

 输入参数：
 - params：动作参数字符串，当前按 `"<siid> <aiid> <params>"` 形式解析。
 - response：输出响应缓冲区
 - resp_max_len：输出缓冲区最大长度

 输出协议（当前实现）：
 - 以 "result" 开头，成功时追加 ` <siid> <aiid> 0`
 - 不支持的动作返回 ` <siid> <aiid> -4003`

 当前实现示例：
 - SYSTEM_SIID(2), aiid==1：重启动作（仅返回成功响应，实际重启逻辑待补齐）
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : build_properties_changed
 Input       : char *payload, int max_len
 Output      : int
 Description :
 根据当前待上报属性标志 `s_prop_flags`，从 `s_device_info` 读取对应字段并构建
 "properties_changed ..." 文本上报 payload。

 输入参数：
 - payload：输出缓冲区，用于存放构建后的上报字符串
 - max_len：payload 最大长度

 构建规则：
 - 仅对 `s_prop_flags` 指示为“发生变化/需要上报”的属性进行拼接；
 - 每个属性追加格式示例：`properties_changed <siid> <piid> <value> ...`
 - 属性字段来源于 `s_device_info`（动态属性）或宏定义（静态属性由其他流程上报）。

 线程安全：
 - 构建过程中会对 `s_device_info` 的读取加 `s_mutex` 互斥，避免并发更新导致不一致。

 返回值约定：
 - 返回值为实际写入 payload 的字符数（snprintf 方式累计 offset）；
 - 若没有任何可上报属性，则可能返回 0。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : mcu_mqtt_callback
 Input       : const char *topic, const char *payload, int len
 Output      : None
 Description :
 MCU 模块的 MQTT 下行统一回调入口。
 该函数由 AT/MQTT 层在收到 "+MQTTURC: \"message\"" 后分发调用（见 `at_parser.c`），
 用于将云端下行消息解析为 MCU 侧可执行的 get/set/action 命令，并在需要时回包。

 输入参数：
 - topic：MQTT 主题字符串
 - payload：消息内容（通常为文本协议或 JSON）
 - len：payload 长度

 处理流程：
 - 调用 `mcu_handle_cloud_message()` 解析并生成 response；
 - 若解析成功且 response 非空，则发布到 `DATA_TOPIC` 回传云端。

 注意：
 - 当前实现将所有响应统一发布到 DATA_TOPIC；若云端协议需要区分 reply topic，
   需在此处根据 topic/协议字段进行路由。
---------------------------------------------------------------------------*/
static void mcu_mqtt_callback(const char *topic, const char *payload, int len) {
    char response[512];
    int ret = mcu_handle_cloud_message(topic, payload, len, response, sizeof(response));
    if (ret == 0 && response[0] != '\0') {
        // 将响应发回云端
        mqtt_client_publish(DATA_TOPIC, response, 0);
    }
}

/*---------------------------------------------------------------------------
 Name        : MCU_Task_Init
 Input       : None
 Output      : None
 Description :
 初始化 MCU 业务处理模块的基础资源与默认设备信息，并注册 MQTT 下行回调。

 初始化内容：
 - 创建互斥锁 `s_mutex`：用于保护 `s_device_info` 与 `s_prop_flags` 等共享数据；
 - 注册 MQTT 消息回调：
   - 通过 `at_mqtt_register_callback(mcu_mqtt_callback)` 将下行消息引入 MCU 处理链路；
 - 初始化默认设备信息 `s_device_info`：
   - 写入 model/sw_version/sn 等静态字段；
   - 设置运行状态/功率/电量/温度/电压/故障码等默认值。

 调用时机：
 - 由 `MCU_Task()` 在任务启动时调用一次。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : MCU_Task
 Input       : void *argument
 Output      : None
 Description :
 MCU 业务主任务：负责属性变化上报、定时心跳上报，以及等待 Cloud 初始化同步点后再开始运行。

 输入参数：
 - argument：RTOS 任务入口参数，当前未使用。

 任务流程（按当前实现）：
 - 调用 `MCU_Task_Init()` 完成资源创建与回调注册；
 - 等待 Cloud 任务完成初始化（MQTT 已连接）：
   - 通过 `task_sync_get_cloud_ready_flag()` 获取事件标志组；
   - 等待 `CLOUD_READY_FLAG_ID` 被设置（不清除标志）。
 - 首次上报：设置 `MCU_PROP_DEVICE_INFO | MCU_PROP_RUN_STATE` 触发上报；
 - 主循环每 100ms 执行：
   - 若 `s_prop_flags != 0`，则调用 `build_properties_changed()` 构建 payload 并发布；
     发布成功后清除 `s_prop_flags`；
   - 否则累计计时，达到 `PROP_REPORT_INTERVAL_MS` 后执行“定期上报”（即使无变化也上报，
     用于保持连接/云端活性）。

 并发说明：
 - 属性变化通常由其他业务模块调用 `mcu_update_device_info()` 或 `mcu_set_prop_changed()`
   设置标志，MCU_Task 负责统一节流与上报发送。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : mcu_set_prop_changed
 Input       : mcu_prop_flag_t prop
 Output      : None
 Description :
 设置“属性需要上报”的标志位，用于触发 MCU_Task 在下一周期构建并发送
 properties_changed 上报。

 输入参数：
 - prop：属性标志位（可按位或组合），类型为 `mcu_prop_flag_t`。

 线程安全：
 - 内部使用 `s_mutex` 保护对 `s_prop_flags` 的读改写，避免并发置位丢失。
---------------------------------------------------------------------------*/
void mcu_set_prop_changed(mcu_prop_flag_t prop) {
    osMutexAcquire(s_mutex, osWaitForever);
    s_prop_flags |= (uint32_t)prop;
    osMutexRelease(s_mutex);
}

/*---------------------------------------------------------------------------
 Name        : mcu_get_device_info
 Input       : None
 Output      : const mcu_device_info_t*
 Description :
 获取当前设备信息结构体的只读指针。

 返回说明：
 - 返回 `s_device_info` 的地址，供只读访问（例如 UI 显示/调试输出）。

 注意：
 - 该函数不加锁；若调用方需要读取一致快照，建议在调用方侧加锁或提供拷贝接口；
 - 若调用方仅做非关键显示，通常可接受读取过程中字段被更新的轻微不一致。
---------------------------------------------------------------------------*/
const mcu_device_info_t* mcu_get_device_info(void) {
    return &s_device_info;
}

/*---------------------------------------------------------------------------
 Name        : mcu_update_device_info
 Input       : const mcu_device_info_t *info
 Output      : None
 Description :
 更新设备动态信息 `s_device_info`，并根据字段变化自动置位对应的属性上报标志。

 输入参数：
 - info：外部采样/计算得到的设备信息（新值），不能为空。

 行为说明：
 - 在互斥保护下逐字段比较 old/new：
   - run_state/power/energy/temperature/voltage/fault_code 等；
 - 对发生变化的字段累计到 changed 标志集合；
 - 将 changed 合并到 `s_prop_flags`，触发 MCU_Task 进行上报。

 设计意图：
 - 将“数据采集/计算”和“上报节流/发送”解耦：
   - 采集侧只负责调用本函数提交新数据；
   - 上报由 MCU_Task 统一在合适的时机发送，避免频繁 publish。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : mcu_handle_cloud_message
 Input       : const char *topic, const char *payload, int len,
               char *response, int resp_max_len
 Output      : int
 Description :
 解析云端下行消息并生成 MCU 侧响应字符串。

 输入参数：
 - topic：MQTT 主题（当前实现未使用，但保留用于未来按 topic 路由/鉴权）
 - payload：消息内容（支持两类格式）
 - len：payload 长度
 - response：输出响应缓冲区
 - resp_max_len：响应缓冲区最大长度

 支持的下行格式：
 1) 文本协议：
    - 形如：`down get_properties ...` / `down set_properties ...` / `down action ...`
 2) JSON 简化协议：
    - 形如：`{"cmd":"get_properties","params":"..."}`
      同理支持 set_properties / action

 处理流程：
 - 将 payload 拷贝到本地可修改缓冲区 buf，并确保 '\0' 终止；
 - 先尝试查找 "down " 前缀走文本协议分支；
 - 若不存在，则尝试从 JSON 中提取 cmd 与 params 字段；
 - 根据命令类型分别调用：
   - `handle_get_properties()`
   - `handle_set_properties()`
   - `handle_action()`
 - 若解析成功，response 中写入以 "result" 开头的响应文本。

 返回值约定：
 - 返回 0：识别并处理成功（response 可能为空或包含结果）
 - 返回 -1：无法识别的格式/命令或缺少关键字段
---------------------------------------------------------------------------*/
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
