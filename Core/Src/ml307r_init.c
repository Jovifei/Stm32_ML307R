#include "ml307r_init.h"
#include "uart_at.h"
#include "at_config.h"
#include "cmsis_os2.h"
#include <string.h>

static ml307r_state_t s_state = ML307R_STATE_INIT;
static signal_quality_t s_sq = {99, 99};
static osMutexId_t s_mutex = NULL;

/*---------------------------------------------------------------------------
 Name        : ml307r_init
 Input       : None
 Output      : int
 Description :
 执行 ML307R 4G 模组的“上电后网络初始化”流程，并在函数内维护模组状态机。
 该流程属于 MCU 全控模式，依赖 `uart_at.c` 提供的 AT 指令收发与超时等待能力。

 主要步骤（按当前实现）：
 - AT 通信探测：发送 "AT" 等待 "OK"，用于确认模组就绪。
 - 关闭回显：发送 "ATE0" 避免回显干扰后续解析。
 - 启用详细错误：发送 "AT+CMEE=1" 便于定位失败原因（CME/CMS）。
 - 关闭睡眠：发送 "AT+CSCLK=0"（调试阶段建议关闭，避免串口休眠导致超时）。
 - SIM 检查：发送 "AT+CPIN?" 并检查响应包含 "READY"。
 - 网络注册：循环查询 "AT+CEREG?"，最多 30 次，每次间隔 1s；
   判定 "+CEREG: 0,1" 或 "+CEREG: 1,1" 为注册成功。
 - PDP 激活/拨号：发送 "AT+MIPCALL=1,1"（长超时）。

 线程安全：
 - 内部使用 `s_mutex` 对 `s_state` 的更新进行互斥保护，避免多任务并发调用时状态竞争。

 返回值约定：
 - 返回 0：初始化并拨号成功，状态进入 ML307R_STATE_CONNECTED
 - 返回 -1：任一步骤失败或超时，状态进入 ML307R_STATE_ERROR
---------------------------------------------------------------------------*/
int ml307r_init(void) {
    int ret;
    char resp[128];

    if (s_mutex == NULL) {
        s_mutex = osMutexNew(NULL);
    }

    osMutexAcquire(s_mutex, osWaitForever);
    s_state = ML307R_STATE_INIT;

    // 1. AT通信测试（等待模组就绪）
    ret = at_send_command("AT", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) { s_state = ML307R_STATE_ERROR; osMutexRelease(s_mutex); return -1; }

    // 2. 关闭回显 ATE0
    ret = at_send_command("ATE0", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) { s_state = ML307R_STATE_ERROR; osMutexRelease(s_mutex); return -1; }

    // 3. 启用详细错误码 +CME ERROR
    ret = at_send_command("AT+CMEE=1", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) { s_state = ML307R_STATE_ERROR; osMutexRelease(s_mutex); return -1; }

    // 4. 关闭睡眠模式（调试阶段建议关闭）
    ret = at_send_command("AT+CSCLK=0", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) { s_state = ML307R_STATE_ERROR; osMutexRelease(s_mutex); return -1; }

    // 3. 检查SIM卡 AT+CPIN?
    s_state = ML307R_STATE_SIM_CHECK;
    ret = at_send_command("AT+CPIN?", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0 || strstr(resp, "READY") == NULL) { s_state = ML307R_STATE_ERROR; osMutexRelease(s_mutex); return -1; }

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

/*---------------------------------------------------------------------------
 Name        : ml307r_get_state
 Input       : None
 Output      : ml307r_state_t
 Description :
 获取 ML307R 模组当前状态机状态 `s_state`。
 - 若互斥锁尚未创建（极早期阶段），直接返回当前 `s_state`。
 - 否则在互斥保护下读取，确保多任务并发下读取一致。

 该接口通常用于：
 - Cloud/OTA/诊断任务根据状态判断是否需要重试初始化/重连；
 - 故障判断（如欠费/注册异常）逻辑的辅助条件。
---------------------------------------------------------------------------*/
ml307r_state_t ml307r_get_state(void) {
    if (s_mutex == NULL) return s_state;
    osMutexAcquire(s_mutex, osWaitForever);
    ml307r_state_t st = s_state;
    osMutexRelease(s_mutex);
    return st;
}

/*---------------------------------------------------------------------------
 Name        : ml307r_get_signal_quality
 Input       : signal_quality_t *sq
 Output      : int
 Description :
 读取并解析模组信号质量（RSSI/BER），并回填到调用者提供的结构体中。

 输入参数：
 - sq：输出结构体指针，不能为空；用于返回解析后的 rssi/ber。

 行为说明：
 - 通过 AT 指令 "AT+CSQ" 获取信号质量；
 - 若指令成功则解析形如 "+CSQ: <rssi>,<ber>" 的内容；
 - 将结果同时写入：
   - 调用者的 `*sq`
   - 内部缓存 `s_sq`（便于后续快速判断/调试）

 返回值约定：
 - 返回 0：成功获取并解析
 - 返回 -1：参数非法或 AT 指令失败/超时
---------------------------------------------------------------------------*/
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
    return ret == 0 ? 0 : -1;
}

/*---------------------------------------------------------------------------
 Name        : ml307r_is_arrears
 Input       : None
 Output      : bool
 Description :
 尝试根据“信号正常但始终无法注册/拨号/连接”的表现，推测模组可能处于欠费/停机等异常状态。

 判定依据（当前实现的经验规则）：
 - 先读取信号质量 `AT+CSQ`：
   - rssi 在 10..31 之间视为信号强度较可用（排除无信号/弱信号导致的注册失败）。
 - 再读取状态机状态：
   - 若状态不在 REGISTERED / DIAL / CONNECTED，则认为网络侧未完成关键阶段。

 注意：
 - 该函数为“启发式”判断，并非运营商侧欠费的确证结果；
 - 可用于上层策略：提示用户、触发更激进的重连、或上报诊断信息。
---------------------------------------------------------------------------*/
bool ml307r_is_arrears(void) {
    signal_quality_t sq;
    ml307r_get_signal_quality(&sq);
    ml307r_state_t st = ml307r_get_state();
    return (sq.rssi >= 10 && sq.rssi <= 31 &&
            st != ML307R_STATE_REGISTERED &&
            st != ML307R_STATE_DIAL &&
            st != ML307R_STATE_CONNECTED);
}

/*---------------------------------------------------------------------------
 Name        : ml307r_reconnect
 Input       : None
 Output      : int
 Description :
 执行 ML307R 的“软重连”流程，用于在网络异常/断线后尝试恢复到已拨号连接状态。

 主要步骤（按当前实现）：
 - 通过 "AT+CFUN=0" 关闭射频功能，短延时后 "AT+CFUN=1" 重新开启；
 - 循环查询 "AT+CEREG?" 等待网络重新注册（最多 30 次，每次 1s）；
 - 注册成功后再次执行 "AT+MIPCALL=1,1" 进行 PDP 激活/拨号；
 - 根据结果更新内部状态机到 REGISTERED/CONNECTED 或 ERROR。

 线程安全与副作用：
 - 该函数会更新全局状态 `s_state`，供其他任务判断当前连网阶段；
 - 当前实现未对整个重连过程加互斥保护（只更新状态变量），
   若可能被多任务并发调用，建议上层保证串行调用或在此处增加互斥包裹。

 返回值约定：
 - 返回 0：重连成功，状态进入 ML307R_STATE_CONNECTED
 - 返回 -1：注册或拨号失败，状态进入 ML307R_STATE_ERROR
---------------------------------------------------------------------------*/
int ml307r_reconnect(void) {
    char resp[128];

    at_send_command("AT+CFUN=0", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    osDelay(500);
    at_send_command("AT+CFUN=1", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    osDelay(2000);

    int retry = 0;
    while (retry < 30) {
        int ret = at_send_command("AT+CEREG?", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
        if (ret == 0 && (strstr(resp, "+CEREG: 0,1") != NULL || strstr(resp, "+CEREG: 1,1") != NULL)) {
            break;
        }
        retry++;
        osDelay(1000);
    }
    if (retry >= 30) { s_state = ML307R_STATE_ERROR; return -1; }

    s_state = ML307R_STATE_REGISTERED;
    int ret = at_send_command("AT+MIPCALL=1,1", "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
    if (ret != 0) { s_state = ML307R_STATE_ERROR; return -1; }

    s_state = ML307R_STATE_CONNECTED;
    return 0;
}
