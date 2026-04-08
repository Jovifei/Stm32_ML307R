#ifndef AT_PARSER_H
#define AT_PARSER_H

#include <stdint.h>
#include <stdbool.h>

typedef void (*mqtt_msg_callback_t)(const char *topic, const char *payload, int payload_len);

// ==================== SSL/TLS证书配置 ====================
// 写入证书到ML307R模块（CA证书/客户端证书）
// cert_data: PEM格式证书数据, cert_len: 证书长度(字节)
// 返回0成功，非0失败
int at_ssl_write_cert(const char *filename, const char *cert_data, uint32_t cert_len);

// 写入客户端私钥到ML307R模块
// key_data: PEM格式私钥数据, key_len: 私钥长度(字节)
// 返回0成功，非0失败
int at_ssl_write_key(const char *filename, const char *key_data, uint32_t key_len);

// 配置SSL认证模式
// auth: 0=无认证, 1=单向认证(仅验证服务器CA), 2=双向认证(mTLS)
int at_ssl_config_auth(uint8_t auth_mode);

// 配置SSL版本 (3=TLS1.2 only, 0=TLS1.0/1.1/1.2)
int at_ssl_config_version(uint8_t version);

// 忽略证书时间戳检查（模块时间不准时设为1）
int at_ssl_config_ignore_timestamp(uint8_t ignore);

// 绑定证书文件到SSL连接
int at_ssl_bind_certs(const char *ca_file, const char *client_file, const char *key_file);

// 完整SSL初始化流程：写入证书+配置参数
// 返回0成功，非0失败
int at_ssl_init_full(void);

// ==================== HTTP/HTTPS请求（用于预配置和证书获取）====================
// HTTP实例ID
#define HTTP_CONN_ID     0

// 创建HTTP实例并配置SSL
// host: 服务器地址（不含https://）
// use_ssl: 1=使用HTTPS, 0=使用HTTP
// 返回0成功，非0失败
int at_http_create(const char *host, uint8_t use_ssl);

// 发送HTTP GET请求
// path: 请求路径（如 "/APIServerV2/tool/mqtt/preset"）
// 返回0成功，非0失败
int at_http_get(const char *path);

// 读取HTTP响应数据
// buf: 接收缓冲区
// buf_size: 缓冲区大小
// actual_len: 实际读取的字节数
// 返回0成功，非0失败
int at_http_read(char *buf, int buf_size, int *actual_len);

// 销毁HTTP实例
void at_http_destroy(void);

// ==================== 预配置：获取device_id/device_key ========================
// 通过HTTPS POST从服务器获取设备凭证
// 使用product_id/product_secret/device_sn计算prov_code=MD5(secret+sn)
// 返回0成功，非0失败
// 成功时将device_id/device_key存储到g_device_id/g_device_key
int at_provision_fetch(void);

// ==================== 证书获取：从服务器获取客户端证书 ========================
// 通过HTTPS GET从服务器获取客户端证书和私钥
// URL: /APIServerV2/tool/cert/<product_id>/<device_id>/<device_key>
// 返回0成功，非0失败
// 成功时将证书写入ML307R模块
int at_cert_fetch_and_write(void);

// ==================== 完整预配置流程（相当于ESP32的https_get_provison + https_get_cert）====
// 依次执行：预配置获取 → 证书获取写入 → SSL初始化
// 返回0成功，非0失败
int at_provision_and_tls_init(void);

// ==================== MQTT配置 ====================
// 配置MQTT参数（SSL绑定在连接参数中）
int at_mqtt_config(const char *host, int port, const char *client_id,
                   const char *username, const char *password);
int at_mqtt_connect(void);
int at_mqtt_disconnect(void);
int at_mqtt_subscribe(const char *topic, int qos);
int at_mqtt_publish(const char *topic, int qos, const char *data, int data_len);

// MQTT消息回调链表管理（支持多模块订阅同一消息）
void at_mqtt_register_callback(mqtt_msg_callback_t callback);
void at_mqtt_unregister_callback(mqtt_msg_callback_t callback);

// ==================== 设备凭证（预配置后填充）====================
extern char g_device_id[32];
extern char g_device_key[32];
extern uint8_t g_provisioned;  // 预配置完成标志

#endif // AT_PARSER_H