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

ml307r_state_t ml307r_get_state(void) {
    if (s_mutex == NULL) return s_state;
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
    return ret == 0 ? 0 : -1;
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
