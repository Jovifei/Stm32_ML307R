#ifndef ML307R_INIT_H
#define ML307R_INIT_H

#include <stdbool.h>

typedef enum
{
    ML307R_STATE_INIT = 0,   // 初始化
    ML307R_STATE_SIM_CHECK,  // SIM卡检查
    ML307R_STATE_REGISTERED, // 网络注册成功
    ML307R_STATE_DIAL,       // PDP拨号中
    ML307R_STATE_CONNECTED,  // 已连接
    ML307R_STATE_ERROR       // 错误状态
} ml307r_state_t;

typedef struct
{
    int rssi; // 0-31, 99=unknown
    int ber;  // 0-7, 99=unknown
} signal_quality_t;

int ml307r_init(void);
ml307r_state_t ml307r_get_state(void);
int ml307r_get_signal_quality(signal_quality_t *sq);
bool ml307r_is_arrears(void);
int ml307r_reconnect(void);

#endif // ML307R_INIT_H
