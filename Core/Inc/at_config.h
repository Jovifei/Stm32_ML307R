#ifndef AT_CONFIG_H
#define AT_CONFIG_H

#include <stdint.h>

// AT命令超时 (ms)
#define AT_TIMEOUT_DEFAULT  3000
#define AT_TIMEOUT_LONG     10000
#define AT_TIMEOUT_CERT     5000    // 证书操作超时

// SSL/TLS配置
#define SSL_ID               0       // SSL配置ID
#define SSL_AUTH_MODE        2       // 0=无认证, 1=单向认证(仅CA), 2=双向认证(mTLS)

// 证书文件名（写入ML307R模块用）
#define SSL_CA_CERT_FILE     "ca.pem"
#define SSL_CLIENT_CERT_FILE "client.pem"
#define SSL_CLIENT_KEY_FILE  "client.key"

// UART RX环形缓冲区大小
#define UART_AT_RX_BUF_SIZE  512

// ==================== EMQ服务器CA证书（用于验证服务器证书）====================
// 来源: D:\work\ESP32_CT\main\iot\emq.c - emq_ca_cert
extern const char g_mqtt_ca_cert[];
extern const char g_mqtt_client_cert[];
extern const char g_mqtt_client_key[];

// 证书数据长度（实际数据在at_config.c中定义）
extern const uint32_t g_mqtt_ca_cert_len;
extern const uint32_t g_mqtt_client_cert_len;
extern const uint32_t g_mqtt_client_key_len;

#endif // AT_CONFIG_H
