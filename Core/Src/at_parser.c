#include "at_parser.h"
#include "uart_at.h"
#include "at_config.h"
#include "config.h"
#include "flash_spi.h"
#include "md5.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define MQTT_CONN_ID 0

// SSL认证模式
#define SSL_AUTH_NONE   0   // 无认证
#define SSL_AUTH_CA     1   // 单向认证(仅验证服务器CA)
#define SSL_AUTH_MTLS   2   // 双向认证(mTLS)

// HTTP配置
#define HTTP_CONN_ID    0
#define HTTP_RESP_TIMEOUT 30000  // HTTP响应超时30秒

// ==================== 设备凭证（预配置后填充）====================
char g_device_id[32] = {0};
char g_device_key[32] = {0};
uint8_t g_provisioned = 0;  // 预配置完成标志

// 凭证存储地址（外部SPI Flash）
#define DEV_ID_ADDR     (PROV_FLASH_ADDR)
#define DEV_KEY_ADDR    (PROV_FLASH_ADDR + 64)
#define CERT_STATE_ADDR (PROV_FLASH_ADDR + 128)

// ==================== 内部工具函数 ====================

// 计算字符串MD5（简化版，用于prov_code计算）
// 注意：ESP32中MD5(product_secret + device_sn)
// 这里复用config.h中已定义的MD5相关常量
static void md5_string(const char *input, char *output) {
    // 简化实现：实际产品中应使用标准MD5算法
    // 此处用字符拼接作为占位，实际需要libmd5或软件MD5实现
    (void)input;
    (void)output;
    // TODO: 实现标准MD5
}

// 从JSON中提取字符串字段
// 返回值: >=0 表示成功提取的字符串长度, <0 表示失败
static int json_extract_str(const char *json, const char *key, char *out, int out_max) {
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"%%31[^\"]\"", key);
    if (sscanf(json, pattern, out) >= 1) {
        return strlen(out);
    }
    snprintf(pattern, sizeof(pattern), "\"%s\":\"\",", key);
    if (strstr(json, pattern) != NULL) {
        out[0] = '\0';
        return 0;
    }
    return -1;
}

// ==================== MQTT回调链表 ====================
#define MQTT_CB_MAX 4
typedef struct {
    mqtt_msg_callback_t cb;
    bool used;
} mqtt_cb_entry_t;
static mqtt_cb_entry_t s_mqtt_cbs[MQTT_CB_MAX];
static bool s_urc_registered = false;

static void mqtt_urc_handler(const char *line) {
    if (line == NULL) return;

    if (strstr(line, "+MQTTURC: \"conn\",0,0") != NULL) return;
    if (strstr(line, "+MQTTURC: \"suback\"") != NULL) return;

    const char *prefix = "+MQTTURC: \"message\"";
    if (strncmp(line, prefix, strlen(prefix)) == 0) {
        char topic[128] = {0};
        char payload[512] = {0};
        int qos = 0, len = 0;
        if (sscanf(line, "+MQTTURC: \"message\",\"%127[^\"]\",%d,%d,\"%511[^\"]\"",
                   topic, &qos, &len, payload) >= 4) {
            int actual_len = len < 512 ? len : 511;
            for (int i = 0; i < MQTT_CB_MAX; i++) {
                if (s_mqtt_cbs[i].used && s_mqtt_cbs[i].cb != NULL) {
                    s_mqtt_cbs[i].cb(topic, payload, actual_len);
                }
            }
        }
    }
}

// ==================== SSL/TLS证书操作 ====================

// 等待证书写入的提示符 '>'
static int wait_cert_prompt(uint32_t timeout_ms) {
    char resp[32];
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout_ms) {
        if (at_send_command("", ">", 100, resp, sizeof(resp)) == 0) {
            return 0;
        }
    }
    return -1;
}

// 写入证书到ML307R模块
int at_ssl_write_cert(const char *filename, const char *cert_data, uint32_t cert_len) {
    if (filename == NULL || cert_data == NULL || cert_len == 0) return -1;

    char cmd[128], resp[128];

    // 分块发送证书（每块最大512字节）
    uint32_t remain = cert_len;
    const char *pData = cert_data;
    uint32_t block_size = 512;

    while (remain > 0) {
        uint32_t chunk = (remain > block_size) ? block_size : remain;
        uint32_t is_last = (chunk >= remain) ? 0 : 1;

        if (is_last) {
            snprintf(cmd, sizeof(cmd), "AT+MSSLCERTWR=\"%s\",0,%u",
                     filename, (unsigned int)chunk);
        } else {
            snprintf(cmd, sizeof(cmd), "AT+MSSLCERTWR=\"%s\",%u,%u",
                     filename, (unsigned int)(remain - chunk), (unsigned int)chunk);
        }

        at_flush_rx();
        char cmd_buf[256];
        snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);
        HAL_UART_Transmit(&huart2, (uint8_t *)cmd_buf, strlen(cmd_buf), 100);

        if (wait_cert_prompt(2000) != 0) {
            // 没有收到'>'，尝试直接发送
        }

        at_send_raw((const uint8_t *)pData, chunk);

        if (at_send_command("", "OK", 3000, resp, sizeof(resp)) != 0) {
            // 可能已经OK
        }

        pData += chunk;
        remain -= chunk;
    }
    return 0;
}

// 写入客户端私钥
int at_ssl_write_key(const char *filename, const char *key_data, uint32_t key_len) {
    if (filename == NULL || key_data == NULL || key_len == 0) return -1;

    char cmd[128], resp[128];
    snprintf(cmd, sizeof(cmd), "AT+MSSLKEYWR=\"%s\",0,%u", filename, (unsigned int)key_len);

    at_flush_rx();
    char cmd_buf[256];
    snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);
    HAL_UART_Transmit(&huart2, (uint8_t *)cmd_buf, strlen(cmd_buf), 100);

    if (wait_cert_prompt(2000) != 0) return -1;

    at_send_raw((const uint8_t *)key_data, key_len);

    if (at_send_command("", "OK", 3000, resp, sizeof(resp)) != 0) return -1;

    return 0;
}

int at_ssl_config_auth(uint8_t auth_mode) {
    char cmd[64], resp[128];
    snprintf(cmd, sizeof(cmd), "AT+MSSLCFG=\"auth\",%d,%d", SSL_ID, auth_mode);
    return at_send_command(cmd, "OK", AT_TIMEOUT_CERT, resp, sizeof(resp));
}

int at_ssl_config_version(uint8_t version) {
    char cmd[64], resp[128];
    snprintf(cmd, sizeof(cmd), "AT+MSSLCFG=\"version\",%d,%d", SSL_ID, version);
    return at_send_command(cmd, "OK", AT_TIMEOUT_CERT, resp, sizeof(resp));
}

int at_ssl_config_ignore_timestamp(uint8_t ignore) {
    char cmd[64], resp[128];
    snprintf(cmd, sizeof(cmd), "AT+MSSLCFG=\"ignorestamp\",%d,%d", SSL_ID, ignore);
    return at_send_command(cmd, "OK", AT_TIMEOUT_CERT, resp, sizeof(resp));
}

int at_ssl_bind_certs(const char *ca_file, const char *client_file, const char *key_file) {
    char cmd[256], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MSSLCFG=\"cert\",%d,\"%s\",\"%s\",\"%s\"",
             SSL_ID, ca_file, client_file, key_file);
    return at_send_command(cmd, "OK", AT_TIMEOUT_CERT, resp, sizeof(resp));
}

// 完整SSL初始化流程（客户端证书从Flash读取）
int at_ssl_init_full(void) {
    int ret;
    char resp[256];
    char client_cert[2048] = {0};
    char client_key[2048] = {0};
    uint32_t cert_len = 0, key_len = 0;

    // 从Flash读取客户端证书
    flash_spi_read(CERT_STATE_ADDR + sizeof(uint32_t), (uint8_t *)client_cert, sizeof(client_cert));
    flash_spi_read(CERT_STATE_ADDR + sizeof(uint32_t) + sizeof(client_cert),
                   (uint8_t *)client_key, sizeof(client_key));
    // 读取证书长度
    flash_spi_read(CERT_STATE_ADDR, (uint8_t *)&cert_len, sizeof(cert_len));
    flash_spi_read(CERT_STATE_ADDR + sizeof(uint32_t) * 2,
                   (uint8_t *)&key_len, sizeof(key_len));

    // 1. 写入CA证书
    if (g_mqtt_ca_cert_len > 0) {
        ret = at_ssl_write_cert(SSL_CA_CERT_FILE, g_mqtt_ca_cert, g_mqtt_ca_cert_len);
        if (ret != 0) return ret;
    }

    // 2. 写入客户端证书
    if (cert_len > 0 && cert_len < sizeof(client_cert)) {
        ret = at_ssl_write_cert(SSL_CLIENT_CERT_FILE, client_cert, cert_len);
        if (ret != 0) return ret;
    }

    // 3. 写入客户端私钥
    if (key_len > 0 && key_len < sizeof(client_key)) {
        ret = at_ssl_write_key(SSL_CLIENT_KEY_FILE, client_key, key_len);
        if (ret != 0) return ret;
    }

    // 4. 配置SSL认证模式
    ret = at_ssl_config_auth(SSL_AUTH_MODE);
    if (ret != 0) return ret;

    // 5. 配置SSL版本（TLS1.2）
    ret = at_ssl_config_version(3);
    if (ret != 0) return ret;

    // 6. 忽略证书时间戳
    ret = at_ssl_config_ignore_timestamp(1);
    if (ret != 0) return ret;

    // 7. 绑定证书文件
    if (cert_len > 0 && key_len > 0) {
        ret = at_ssl_bind_certs(SSL_CA_CERT_FILE, SSL_CLIENT_CERT_FILE, SSL_CLIENT_KEY_FILE);
    } else {
        ret = at_ssl_bind_certs(SSL_CA_CERT_FILE, "", "");
    }

    return ret;
}

// ==================== HTTP/HTTPS请求 ====================

// 创建HTTP实例并配置SSL
int at_http_create(const char *host, uint8_t use_ssl) {
    char cmd[128], resp[128];
    int ret;

    // 创建HTTP实例
    snprintf(cmd, sizeof(cmd), "AT+MHTTPCREATE=\"https://%s\"", host);
    ret = at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
    if (ret != 0) {
        // 尝试HTTP
        snprintf(cmd, sizeof(cmd), "AT+MHTTPCREATE=\"http://%s\"", host);
        ret = at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
        if (ret != 0) return ret;
    }

    if (use_ssl) {
        // 配置SSL使能
        snprintf(cmd, sizeof(cmd), "AT+MHTTPCFG=\"ssl\",%d,1,%d", HTTP_CONN_ID, SSL_ID);
        ret = at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
        if (ret != 0) return ret;
    }

    // 配置超时
    snprintf(cmd, sizeof(cmd), "AT+MHTTPCFG=\"timeout\",%d,30,60", HTTP_CONN_ID);
    ret = at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
    if (ret != 0) return ret;

    return 0;
}

// 发送HTTP GET请求
int at_http_get(const char *path) {
    char cmd[256], resp[128];
    snprintf(cmd, sizeof(cmd), "AT+MHTTPREQ=%d,\"GET\",\"%s\",0", HTTP_CONN_ID, path);
    return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

// 读取HTTP响应（分块读取）
int at_http_read(char *buf, int buf_size, int *actual_len) {
    char cmd[64], resp[2048];
    int ret, offset = 0;

    *actual_len = 0;
    memset(buf, 0, buf_size);

    while (offset < buf_size - 1) {
        int chunk = (buf_size - offset - 1) > 2048 ? 2048 : (buf_size - offset - 1);
        snprintf(cmd, sizeof(cmd), "AT+MHTTPRD=%d,%d", HTTP_CONN_ID, chunk);

        at_flush_rx();
        ret = at_send_command(cmd, "+MHTTPRD:", 5000, resp, sizeof(resp));
        if (ret != 0) {
            // 没有更多数据
            break;
        }

        // 解析 +MHTTPRD: <len>,<data>
        char *comma = strchr(resp, ',');
        if (comma) {
            int data_len = atoi(comma + 1);
            if (data_len > 0 && data_len < chunk) {
                char *data_start = comma + 1;
                // 找到数据起始位置（可能有\r\n前缀）
                while (*data_start == '\r' || *data_start == '\n' || *data_start == ' ') {
                    data_start++;
                }
                int copy_len = strlen(data_start);
                if (copy_len > data_len) copy_len = data_len;
                if (copy_len > 0 && offset + copy_len < buf_size) {
                    memcpy(buf + offset, data_start, copy_len);
                    offset += copy_len;
                }
            }
        }

        // 检查是否还有数据（简单方法：看响应是否只包含OK）
        if (strstr(resp, "OK") && strlen(resp) < 20) {
            break;
        }
    }

    *actual_len = offset;
    return 0;
}

// 销毁HTTP实例
void at_http_destroy(void) {
    char cmd[64], resp[128];
    snprintf(cmd, sizeof(cmd), "AT+MHTTPDESTROY=%d", HTTP_CONN_ID);
    at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

// ==================== 预配置：获取device_id/device_key ====================
// ESP32 equivalent: https_get_provison()
// POST /APIServerV2/tool/mqtt/preset
// Body: {"product_id":"xxx","sn":"xxx","prov_code":"xxx","mark":"xxx"}
// Response: {"data": {"device_id":"xxx","device_key":"xxx"}}

int at_provision_fetch(void) {
    int ret;
    char resp[1024];
    char body[256];
    char prov_code[32] = {0};
    char md5_input[64] = {0};

    // 计算prov_code = MD5(product_secret + device_sn)
    // ESP32: md5InputBuf = product_secret + device_sn
    //        MD5_Encryption(md5InputBuf, md5CalcResult);
    snprintf(md5_input, sizeof(md5_input), "%s%s",
             DEVICE_PRODUCT_SECRET, DEVICE_SN);
    MD5_Encryption(md5_input, prov_code);

    // 构造请求体
    snprintf(body, sizeof(body),
             "{\"product_id\":\"%s\",\"sn\":\"%s\",\"prov_code\":\"%s\",\"mark\":\"%s\"}",
             DEVICE_PRODUCT_ID, DEVICE_SN, prov_code, DEVICE_COUNTRY);

    // 创建HTTP连接
    ret = at_http_create(API_SERVER, 1);  // HTTPS
    if (ret != 0) return ret;

    // 发送POST请求
    char cmd[512], http_body[512];
    snprintf(cmd, sizeof(cmd),
             "AT+MHTTPREQ=%d,\"POST\",\"%s\",0,%d,\"%s\"",
             HTTP_CONN_ID, PROVISION_PATH, (int)strlen(body), body);

    at_flush_rx();
    ret = at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
    if (ret != 0) {
        at_http_destroy();
        return ret;
    }

    // 读取响应
    char json_buf[1024];
    int json_len = 0;
    ret = at_http_read(json_buf, sizeof(json_buf), &json_len);
    at_http_destroy();
    if (ret != 0 || json_len == 0) return -1;

    // 解析JSON提取device_id和device_key
    char did[32] = {0}, dkey[32] = {0};
    if (json_extract_str(json_buf, "device_id", did, sizeof(did)) < 0 ||
        json_extract_str(json_buf, "device_key", dkey, sizeof(dkey)) < 0) {
        return -1;
    }

    // 保存到全局变量
    strncpy(g_device_id, did, sizeof(g_device_id) - 1);
    strncpy(g_device_key, dkey, sizeof(g_device_key) - 1);
    g_provisioned = 1;

    // 写入外部Flash持久化存储
    flash_spi_write(DEV_ID_ADDR, (uint8_t *)g_device_id, strlen(g_device_id));
    flash_spi_write(DEV_KEY_ADDR, (uint8_t *)g_device_key, strlen(g_device_key));

    return 0;
}

// ==================== 证书获取：获取客户端证书并写入模块 ====================
// ESP32 equivalent: https_get_cert()
// GET /APIServerV2/tool/cert/<product_id>/<device_id>/<device_key>
// Response: {"data": {"key":"<PEM>","cert":"<PEM>"}}

int at_cert_fetch_and_write(void) {
    int ret;
    char resp[2048];
    char path[256];

    // 构建证书请求URL
    snprintf(path, sizeof(path), "%s/%s/%s/%s",
             CERT_PATH, DEVICE_PRODUCT_ID, g_device_id, g_device_key);

    // 创建HTTP连接
    ret = at_http_create(API_SERVER, 1);  // HTTPS
    if (ret != 0) return ret;

    // 发送GET请求
    ret = at_http_get(path);
    if (ret != 0) {
        at_http_destroy();
        return ret;
    }

    // 读取响应
    char json_buf[4096];
    int json_len = 0;
    ret = at_http_read(json_buf, sizeof(json_buf), &json_len);
    at_http_destroy();
    if (ret != 0 || json_len == 0) return -1;

    // 解析JSON提取cert和key
    char cert_buf[2048] = {0}, key_buf[2048] = {0};
    if (json_extract_str(json_buf, "cert", cert_buf, sizeof(cert_buf)) < 0 ||
        json_extract_str(json_buf, "key", key_buf, sizeof(key_buf)) < 0) {
        return -1;
    }

    uint32_t cert_len = strlen(cert_buf);
    uint32_t key_len = strlen(key_buf);

    // 存储到外部Flash
    flash_spi_write(CERT_STATE_ADDR, (uint8_t *)&cert_len, sizeof(cert_len));
    flash_spi_write(CERT_STATE_ADDR + sizeof(uint32_t), (uint8_t *)cert_buf, cert_len);
    flash_spi_write(CERT_STATE_ADDR + sizeof(uint32_t) * 2, (uint8_t *)&key_len, sizeof(key_len));
    flash_spi_write(CERT_STATE_ADDR + sizeof(uint32_t) + sizeof(cert_buf),
                    (uint8_t *)key_buf, key_len);

    return 0;
}

// ==================== 完整预配置流程 ====================
int at_provision_and_tls_init(void) {
    int ret;

    // 从Flash尝试加载已存储的凭证
    char stored_id[32] = {0}, stored_key[32] = {0};
    flash_spi_read(DEV_ID_ADDR, (uint8_t *)stored_id, sizeof(stored_id));
    flash_spi_read(DEV_KEY_ADDR, (uint8_t *)stored_key, sizeof(stored_key));

    if (stored_id[0] != '\0' && stored_key[0] != '\0') {
        // 已有有效凭证，加载使用
        strncpy(g_device_id, stored_id, sizeof(g_device_id) - 1);
        strncpy(g_device_key, stored_key, sizeof(g_device_key) - 1);
        g_provisioned = 1;
    } else {
        // 无凭证，需要从服务器获取
        ret = at_provision_fetch();
        if (ret != 0) return ret;

        // 获取证书
        ret = at_cert_fetch_and_write();
        if (ret != 0) return ret;
    }

    return 0;
}

// ==================== MQTT配置 ====================

int at_mqtt_config(const char *host, int port, const char *client_id,
                   const char *username, const char *password) {
    char cmd[512], resp[256];
    snprintf(cmd, sizeof(cmd),
             "AT+MQTTCONN=%d,\"%s\",%d,\"%s\",\"%s\",\"%s\"",
             MQTT_CONN_ID, host, port, client_id, username, password);
    return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

int at_mqtt_connect(void) {
    char cmd[64], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=%d", MQTT_CONN_ID);
    return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

int at_mqtt_disconnect(void) {
    char cmd[64], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTDISC=%d", MQTT_CONN_ID);
    return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

int at_mqtt_subscribe(const char *topic, int qos) {
    char cmd[256], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=%d,\"%s\",%d", MQTT_CONN_ID, topic, qos);
    return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

int at_mqtt_publish(const char *topic, int qos, const char *data, int data_len) {
    if (data == NULL || data_len == 0) data_len = strlen(data);
    char cmd[512], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTPUB=%d,\"%s\",%d,0,0,%d,\"%s\"",
             MQTT_CONN_ID, topic, qos, data_len, data);
    return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

void at_mqtt_register_callback(mqtt_msg_callback_t callback) {
    if (callback == NULL) return;

    for (int i = 0; i < MQTT_CB_MAX; i++) {
        if (!s_mqtt_cbs[i].used) {
            s_mqtt_cbs[i].cb = callback;
            s_mqtt_cbs[i].used = true;
            break;
        }
    }

    if (!s_urc_registered) {
        at_register_urc("+MQTTURC:", mqtt_urc_handler);
        s_urc_registered = true;
    }
}

void at_mqtt_unregister_callback(mqtt_msg_callback_t callback) {
    if (callback == NULL) return;
    for (int i = 0; i < MQTT_CB_MAX; i++) {
        if (s_mqtt_cbs[i].used && s_mqtt_cbs[i].cb == callback) {
            s_mqtt_cbs[i].used = false;
            s_mqtt_cbs[i].cb = NULL;
            break;
        }
    }
}