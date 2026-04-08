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

// 从JSON中提取字符串字段
// 返回值: >=0 表示成功提取的字符串长度, <0 表示失败
/*---------------------------------------------------------------------------
 Name        : json_extract_str
 Input       : const char *json, const char *key, char *out, int out_max
 Output      : int
 Description :
 从 JSON 文本中按 key 提取对应的字符串字段值（简化解析器，基于 sscanf/字符串匹配）。

 输入参数：
 - json：JSON 原始文本（必须为 '\\0' 结尾字符串）。
 - key：要提取的字段名（不含引号）。
 - out：输出缓冲区，用于存放提取到的字符串内容（不含外层双引号）。
 - out_max：输出缓冲区最大长度（当前实现未严格使用该参数限制 sscanf 写入长度）。

 支持的 JSON 片段形式（当前实现）：
 - `"key":"value"`：成功提取 value
 - `"key":"",`：认为字段存在但为空字符串，返回 0

 返回值约定：
 - >=0：成功提取到的字符串长度（空字符串返回 0）
 - -1：未找到字段或解析失败

 注意事项与局限：
 - 非通用 JSON 解析器：不支持转义字符、嵌套对象、数组、空白多样化等复杂情况；
 - `out_max` 未参与格式化字符串的宽度控制（此处用 "%31[^\"]" 固定上限），
   若 out 缓冲区小于 32 字节可能存在风险；建议调用者为 out 分配 >=32 字节或改进实现。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : mqtt_urc_handler
 Input       : const char *line
 Output      : None
 Description :
 MQTT URC（Unsolicited Result Code）行处理函数：解析模组上报的 MQTT 事件，
 并在收到消息时分发给已注册的业务回调。

 输入参数：
 - line：一行以 '\\0' 结尾的 URC 文本（来自 AT 接收解析层）。

 处理内容（按当前实现）：
 - 忽略连接成功提示：`+MQTTURC: "conn",0,0`
 - 忽略订阅确认提示：`+MQTTURC: "suback"`
 - 解析消息提示：
   `+MQTTURC: "message","<topic>",<qos>,<len>,"<data>"`
   解析后将 topic/payload/len 分发给 `s_mqtt_cbs[]` 中所有已登记的回调。

 注意事项：
 - payload 使用 "%511[^\"]" 读取，无法包含双引号且长度上限 511；
 - URC 行格式若变化或 payload 含转义字符，当前 sscanf 解析可能失败；
 - 函数不做耗时操作，适合作为 URC 回调直接执行（但回调链上的业务处理需注意时延）。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : wait_cert_prompt
 Input       : uint32_t timeout_ms
 Output      : int
 Description :
 在向 ML307R 写入证书/私钥时，AT 命令通常会返回一个提示符 '>' 表示可以开始发送原始数据。
 本函数用于在给定超时时间内轮询等待该提示符出现。

 输入参数：
 - timeout_ms：等待超时（毫秒）

 实现策略：
 - 以 `HAL_GetTick()` 为时间基准循环；
 - 周期性调用 `at_send_command("", ">", 100, ...)` 尝试匹配 '>'；
 - 超时返回失败。

 返回值约定：
 - 返回 0：在超时前收到 '>' 提示符
 - 返回 -1：超时未收到提示符

 注意：
 - 这里用空命令触发 `at_send_command` 读取/匹配机制，依赖 `uart_at.c` 的实现细节；
 - 若底层 `at_send_command` 并不支持期望 '>' 的匹配，则应改为专用读缓冲扫描实现。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : at_ssl_write_cert
 Input       : const char *filename, const char *cert_data, uint32_t cert_len
 Output      : int
 Description :
 将 CA/客户端证书（PEM 文本）写入 ML307R 模组的证书存储区，供后续 TLS 连接使用。

 输入参数：
 - filename：模组内部证书文件名（例如 `SSL_CA_CERT_FILE` 等）
 - cert_data：证书数据（通常为 PEM 格式 ASCII 文本）
 - cert_len：证书数据长度（字节）

 分块写入策略（按当前实现）：
 - 以 512 字节为一块循环写入；
 - 每块先发送 `AT+MSSLCERTWR="file",<offset>,<len>` 并等待 '>' 提示符；
 - 收到 '>' 后用 `at_send_raw()` 发送该块数据；
 - 然后等待 "OK" 作为写入确认（部分情况下实现允许直接继续）。

 返回值约定：
 - 返回 0：完成全部块写入（不代表逐块都严格确认成功，当前实现容错较宽）
 - 返回 -1：参数非法或关键步骤失败（例如等不到 '>'）

 风险与注意事项：
 - 当前实现对 "OK" 的校验容错较大：某些分支失败不会立即返回；
 - 若证书内容包含二进制或超长数据，需确认模组命令对长度/换行的要求；
 - `huart2` 在此文件中直接使用，依赖外部定义（见 `uart_at.c` 约定）。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : at_ssl_write_key
 Input       : const char *filename, const char *key_data, uint32_t key_len
 Output      : int
 Description :
 将客户端私钥（PEM 文本）写入 ML307R 模组，用于 mTLS（双向认证）场景。

 输入参数：
 - filename：模组内部私钥文件名（例如 `SSL_CLIENT_KEY_FILE`）
 - key_data：私钥数据（PEM 文本）
 - key_len：私钥长度（字节）

 通信序列（按当前实现）：
 - 发送 `AT+MSSLKEYWR="file",0,<len>` 并等待 '>' 提示符；
 - 收到提示符后发送 key_data 原始数据；
 - 等待 "OK" 作为写入成功标志。

 返回值约定：
 - 返回 0：写入成功
 - 返回 -1：参数非法、等待提示符失败或等待 OK 失败
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : at_ssl_config_auth
 Input       : uint8_t auth_mode
 Output      : int
 Description :
 配置 ML307R 的 SSL 认证模式（无认证 / 单向认证 / 双向认证）。

 输入参数：
 - auth_mode：认证模式枚举值
   - 0：无认证（SSL_AUTH_NONE）
   - 1：单向认证，仅验证服务器 CA（SSL_AUTH_CA）
   - 2：双向认证 mTLS（SSL_AUTH_MTLS）

 底层命令：
 - `AT+MSSLCFG="auth",<SSL_ID>,<auth_mode>`

 返回值约定：
 - 返回 0：配置成功（OK）
 - 返回非 0：配置失败/超时（由 `at_send_command` 返回）
---------------------------------------------------------------------------*/
int at_ssl_config_auth(uint8_t auth_mode) {
    char cmd[64], resp[128];
    snprintf(cmd, sizeof(cmd), "AT+MSSLCFG=\"auth\",%d,%d", SSL_ID, auth_mode);
    return at_send_command(cmd, "OK", AT_TIMEOUT_CERT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_ssl_config_version
 Input       : uint8_t version
 Output      : int
 Description :
 配置 ML307R 的 TLS/SSL 协议版本参数。

 输入参数：
 - version：版本值（具体取值需参考模组 AT 手册）
   - 当前工程在 `at_ssl_init_full()` 中使用 3 表示 TLS1.2（示例/经验值）。

 底层命令：
 - `AT+MSSLCFG="version",<SSL_ID>,<version>`

 返回值约定：
 - 返回 0：配置成功
 - 返回非 0：配置失败/超时
---------------------------------------------------------------------------*/
int at_ssl_config_version(uint8_t version) {
    char cmd[64], resp[128];
    snprintf(cmd, sizeof(cmd), "AT+MSSLCFG=\"version\",%d,%d", SSL_ID, version);
    return at_send_command(cmd, "OK", AT_TIMEOUT_CERT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_ssl_config_ignore_timestamp
 Input       : uint8_t ignore
 Output      : int
 Description :
 配置是否忽略证书的时间戳校验（证书有效期）。

 输入参数：
 - ignore：0=不忽略（严格校验）；1=忽略（放宽校验）

 使用场景：
 - 设备没有可靠 RTC/网络授时，导致证书时间校验失败时可临时开启忽略；
 - 安全上建议在量产/正式版本中尽量避免忽略时间戳。

 底层命令：
 - `AT+MSSLCFG="ignorestamp",<SSL_ID>,<ignore>`

 返回值约定：
 - 返回 0：配置成功
 - 返回非 0：配置失败/超时
---------------------------------------------------------------------------*/
int at_ssl_config_ignore_timestamp(uint8_t ignore) {
    char cmd[64], resp[128];
    snprintf(cmd, sizeof(cmd), "AT+MSSLCFG=\"ignorestamp\",%d,%d", SSL_ID, ignore);
    return at_send_command(cmd, "OK", AT_TIMEOUT_CERT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_ssl_bind_certs
 Input       : const char *ca_file, const char *client_file, const char *key_file
 Output      : int
 Description :
 将 CA 证书/客户端证书/私钥文件名绑定到指定 SSL 配置实例（SSL_ID），让后续 TLS 连接使用。

 输入参数：
 - ca_file：CA 证书文件名（必填通常）
 - client_file：客户端证书文件名（mTLS 时需要；单向认证可为空字符串）
 - key_file：客户端私钥文件名（mTLS 时需要；单向认证可为空字符串）

 底层命令：
 - `AT+MSSLCFG="cert",<SSL_ID>,"<ca>","<client>","<key>"`

 返回值约定：
 - 返回 0：绑定成功
 - 返回非 0：绑定失败/超时
---------------------------------------------------------------------------*/
int at_ssl_bind_certs(const char *ca_file, const char *client_file, const char *key_file) {
    char cmd[256], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MSSLCFG=\"cert\",%d,\"%s\",\"%s\",\"%s\"",
             SSL_ID, ca_file, client_file, key_file);
    return at_send_command(cmd, "OK", AT_TIMEOUT_CERT, resp, sizeof(resp));
}

// 完整SSL初始化流程（客户端证书从Flash读取）
/*---------------------------------------------------------------------------
 Name        : at_ssl_init_full
 Input       : None
 Output      : int
 Description :
 执行完整的 TLS 初始化流程：从外部 SPI Flash 读取客户端证书/私钥（若有），并写入 ML307R；
 同时写入 CA 证书，并完成 SSL 参数配置与证书绑定。

 数据来源：
 - CA 证书：`g_mqtt_ca_cert` / `g_mqtt_ca_cert_len`（通常编译期内置）
 - 客户端证书/私钥：从外部 Flash 的 `CERT_STATE_ADDR` 读取（由预配置/拉证书流程写入）

 流程步骤（按当前实现）：
 - 读取 cert_len / key_len 及对应 PEM 内容；
 - 写入 CA 证书（若长度>0）；
 - 写入客户端证书（若 cert_len 有效）；
 - 写入客户端私钥（若 key_len 有效）；
 - 配置认证模式（`at_ssl_config_auth(SSL_AUTH_MODE)`）；
 - 配置版本（示例：TLS1.2 -> version=3）；
 - 配置忽略时间戳（ignore=1）；
 - 绑定证书文件：
   - 若证书/私钥均存在：绑定 CA+client+key
   - 否则：仅绑定 CA（单向认证）

 返回值约定：
 - 返回 0：全部步骤成功
 - 返回非 0：任一步骤失败（向上传递失败码）

 注意事项：
 - 读取外部 Flash 时读取了固定大小缓冲区（2048），真实证书超长时会被截断；
 - `CERT_STATE_ADDR` 的布局需与写入端一致，否则长度解析可能错误。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : at_http_create
 Input       : const char *host, uint8_t use_ssl
 Output      : int
 Description :
 创建 ML307R 的 HTTP 客户端实例，并按需要启用 SSL/TLS，同时配置超时参数。

 输入参数：
 - host：目标主机域名或 IP（例如 API_SERVER）
 - use_ssl：是否启用 SSL（1=启用，0=不启用）

 行为说明：
 - 优先尝试使用 https:// 创建：
   - `AT+MHTTPCREATE="https://<host>"`
 - 若失败，退回尝试 http://：
   - `AT+MHTTPCREATE="http://<host>"`
 - 若 use_ssl==1，则配置：
   - `AT+MHTTPCFG="ssl",<HTTP_CONN_ID>,1,<SSL_ID>`
 - 配置超时：
   - `AT+MHTTPCFG="timeout",<HTTP_CONN_ID>,30,60`

 返回值约定：
 - 返回 0：创建与配置成功
 - 返回非 0：任一步骤失败（返回底层 AT 失败码）
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : at_http_get
 Input       : const char *path
 Output      : int
 Description :
 发送 HTTP GET 请求到已创建的 HTTP 实例（HTTP_CONN_ID）。

 输入参数：
 - path：请求路径（例如 "/APIServerV2/..."），由 `at_http_create()` 中的 base URL 拼接。

 底层命令：
 - `AT+MHTTPREQ=<id>,"GET","<path>",0`

 返回值约定：
 - 返回 0：命令发送成功并收到 OK
 - 返回非 0：发送失败/超时
---------------------------------------------------------------------------*/
int at_http_get(const char *path) {
    char cmd[256], resp[128];
    snprintf(cmd, sizeof(cmd), "AT+MHTTPREQ=%d,\"GET\",\"%s\",0", HTTP_CONN_ID, path);
    return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

// 读取HTTP响应（分块读取）
/*---------------------------------------------------------------------------
 Name        : at_http_read
 Input       : char *buf, int buf_size, int *actual_len
 Output      : int
 Description :
 读取 ML307R HTTP 响应数据到调用者缓冲区，采用分块循环读取方式。

 输入参数：
 - buf：输出缓冲区
 - buf_size：输出缓冲区大小
 - actual_len：输出实际拷贝长度

 行为说明（按当前实现）：
 - 每次最多读取 2048 字节：
   - 发送 `AT+MHTTPRD=<id>,<chunk>`
 - 期望响应包含 `+MHTTPRD:` 前缀；
 - 简单解析返回字符串中逗号后的长度字段与数据起始位置，并复制到 buf；
 - 当读取失败或判断“无更多数据”时退出循环。

 返回值约定：
 - 返回 0：读取流程完成（即使只读到部分数据也返回 0）
 - （当前实现未区分错误返回，读取失败多以 break 结束）

 注意事项/局限：
 - `at_send_command` 当前实现只严格识别 OK，expected 参数可能被忽略（依赖 `uart_at.c`），
   因此这里用 "+MHTTPRD:" 作为期望值可能不生效；
 - 对 `+MHTTPRD:` 的解析不严格，`atoi(comma+1)` 并不等价于 `<len>` 字段；
 - 若 HTTP 响应为二进制或含 '\\0'，使用 strlen/字符串处理会截断；
 - 建议后续改为按协议解析长度字段并按长度拷贝原始数据。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : at_http_destroy
 Input       : None
 Output      : None
 Description :
 销毁 ML307R 的 HTTP 客户端实例，释放模组内部资源。

 底层命令：
 - `AT+MHTTPDESTROY=<HTTP_CONN_ID>`

 设计说明：
 - 该函数不向上返回错误码，作为“尽力清理”的接口；
 - 通常在一次 HTTPS/HTTP 事务完成后调用，避免占用模组连接资源。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : at_provision_fetch
 Input       : None
 Output      : int
 Description :
 执行“预配置”流程：通过 HTTPS 请求云端接口获取 device_id/device_key，
 并将凭证保存到全局变量与外部 SPI Flash（持久化）。

 业务背景：
 - 设备首次上线时需要向服务器进行身份初始化，获取用于 MQTT 鉴权的 device_id/device_key；
 - prov_code 通常用于证明设备与产品的绑定关系（示例为 MD5(product_secret + SN)）。

 流程步骤（按当前实现）：
 - 计算 prov_code：
   - 使用 `MD5_Encryption(product_secret + device_sn, prov_code)`；
 - 构建 POST body：
   - {"product_id":...,"sn":...,"prov_code":...,"mark":...}
 - 创建 HTTPS 连接：`at_http_create(API_SERVER, 1)`
 - 发送 POST 请求：`AT+MHTTPREQ=...,"POST",...`
 - 读取响应 JSON：`at_http_read()`
 - 从 JSON 提取 `device_id`、`device_key`（`json_extract_str`）
 - 写入：
   - 全局变量 `g_device_id` / `g_device_key`，并置 `g_provisioned=1`
   - 外部 Flash：`flash_spi_write(DEV_ID_ADDR, ...)` / `flash_spi_write(DEV_KEY_ADDR, ...)`

 返回值约定：
 - 返回 0：成功获取并保存凭证
 - 返回非 0：HTTP/解析失败等（当前实现多返回 -1 或透传 AT 返回值）

 注意事项：
 - Flash 写入未先擦除，且未保存长度/校验；再次上电读取时可能存在残留数据风险；
 - `json_extract_str` 为简化解析器，对响应格式依赖较强。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : at_cert_fetch_and_write
 Input       : None
 Output      : int
 Description :
 从云端接口拉取设备的客户端证书与私钥（PEM），并写入外部 SPI Flash 做持久化存储。

 输入依赖：
 - `g_device_id` / `g_device_key` 必须已通过预配置获取并有效；
 - `DEVICE_PRODUCT_ID` 用于拼接证书请求路径。

 流程步骤（按当前实现）：
 - 构建 GET 路径：`CERT_PATH/<product_id>/<device_id>/<device_key>`
 - 创建 HTTPS 连接：`at_http_create(API_SERVER, 1)`
 - 发送 GET：`at_http_get(path)`
 - 读取响应 JSON：`at_http_read()`
 - 从 JSON 提取：
   - cert（客户端证书 PEM）
   - key（客户端私钥 PEM）
 - 计算长度并写入外部 Flash（CERT_STATE_ADDR 布局）：
   - cert_len（uint32_t）
   - cert_buf（固定 2048 区域）
   - key_len（uint32_t）
   - key_buf（固定 2048 区域，写入位置为 CERT_STATE_ADDR + sizeof(uint32_t) + sizeof(cert_buf)）

 返回值约定：
 - 返回 0：成功获取并写入 Flash
 - 返回非 0：HTTP/解析/写入失败

 注意事项：
 - 当前函数名含 "write"，但实际并未直接写入 ML307R 模组证书区；
   写入模组的动作由 `at_ssl_init_full()` 完成（从 Flash 再读出并写入模组）。
 - cert/key 缓冲区固定 2048，超长 PEM 会被截断，需与服务端证书长度约束匹配。
---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------
 Name        : at_provision_and_tls_init
 Input       : None
 Output      : int
 Description :
 执行完整的“预配置”与 TLS 前置准备流程：
 - 优先从外部 Flash 加载已持久化的 device_id/device_key；
 - 若不存在有效凭证，则在线拉取凭证并拉取证书写入 Flash；
 - 该函数不直接执行 TLS 参数配置/证书写入模组，属于“准备阶段”的封装入口。

 流程步骤（按当前实现）：
 - 从 Flash 读取 stored_id/stored_key；
 - 若两者均非空：
   - 复制到 `g_device_id/g_device_key`，置 `g_provisioned=1`
 - 否则：
   - 调用 `at_provision_fetch()` 在线获取凭证并保存
   - 调用 `at_cert_fetch_and_write()` 在线获取证书/私钥并保存到 Flash

 返回值约定：
 - 返回 0：准备成功（已有或成功拉取并存储凭证/证书）
 - 返回非 0：任一步失败

 典型调用方：
 - `mqtt_client_connect()` 在建立 MQTT 连接前调用，用于保证鉴权材料齐全。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : at_mqtt_config
 Input       : const char *host, int port, const char *client_id,
               const char *username, const char *password
 Output      : int
 Description :
 配置 ML307R 的 MQTT 连接参数（服务器、端口、客户端标识与鉴权信息）。

 输入参数：
 - host：MQTT Broker 地址（域名或 IP）
 - port：MQTT 端口
 - client_id：客户端 ID（当前工程通常使用 `DEVICE_SN`）
 - username/password：鉴权信息（当前工程使用 provision 得到的 device_id/device_key）

 底层命令：
 - `AT+MQTTCONN=<id>,"<host>",<port>,"<client_id>","<username>","<password>"`

 返回值约定：
 - 返回 0：配置成功（OK）
 - 返回非 0：配置失败/超时
---------------------------------------------------------------------------*/
int at_mqtt_config(const char *host, int port, const char *client_id,
                   const char *username, const char *password) {
    char cmd[512], resp[256];
    snprintf(cmd, sizeof(cmd),
             "AT+MQTTCONN=%d,\"%s\",%d,\"%s\",\"%s\",\"%s\"",
             MQTT_CONN_ID, host, port, client_id, username, password);
    return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_connect
 Input       : None
 Output      : int
 Description :
 发起 MQTT 连接建立（基于已配置的连接参数）。

 底层命令：
 - `AT+MQTTCONN=<id>`

 返回值约定：
 - 返回 0：命令执行成功并返回 OK（不代表一定已经完成所有握手，具体由模组实现决定）
 - 返回非 0：失败/超时
---------------------------------------------------------------------------*/
int at_mqtt_connect(void) {
    char cmd[64], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=%d", MQTT_CONN_ID);
    return at_send_command(cmd, "OK", AT_TIMEOUT_LONG, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_disconnect
 Input       : None
 Output      : int
 Description :
 断开 MQTT 连接。

 底层命令：
 - `AT+MQTTDISC=<id>`

 返回值约定：
 - 返回 0：断开成功
 - 返回非 0：断开失败/超时
---------------------------------------------------------------------------*/
int at_mqtt_disconnect(void) {
    char cmd[64], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTDISC=%d", MQTT_CONN_ID);
    return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_subscribe
 Input       : const char *topic, int qos
 Output      : int
 Description :
 订阅指定 MQTT 主题，用于接收云端下行消息。

 输入参数：
 - topic：主题字符串
 - qos：订阅 QoS 等级

 底层命令：
 - `AT+MQTTSUB=<id>,"<topic>",<qos>`

 返回值约定：
 - 返回 0：订阅命令成功（OK）
 - 返回非 0：失败/超时
---------------------------------------------------------------------------*/
int at_mqtt_subscribe(const char *topic, int qos) {
    char cmd[256], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=%d,\"%s\",%d", MQTT_CONN_ID, topic, qos);
    return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_publish
 Input       : const char *topic, int qos, const char *data, int data_len
 Output      : int
 Description :
 发布 MQTT 消息到指定主题。

 输入参数：
 - topic：发布主题
 - qos：QoS 等级
 - data：消息内容字符串（当前实现按字符串拼接进 AT 命令）
 - data_len：消息长度；若为 0，则使用 strlen(data) 计算

 底层命令（当前实现）：
 - `AT+MQTTPUB=<id>,"<topic>",<qos>,0,0,<len>,"<data>"`

 返回值约定：
 - 返回 0：发布成功（OK）
 - 返回非 0：失败/超时

 注意事项：
 - data 会被放入双引号中，若包含双引号或特殊字符可能导致 AT 命令语法错误；
 - 对二进制 payload 不适用，需改用模组的二进制发布接口（若支持）。
---------------------------------------------------------------------------*/
int at_mqtt_publish(const char *topic, int qos, const char *data, int data_len) {
    if (data == NULL || data_len == 0) data_len = strlen(data);
    char cmd[512], resp[256];
    snprintf(cmd, sizeof(cmd), "AT+MQTTPUB=%d,\"%s\",%d,0,0,%d,\"%s\"",
             MQTT_CONN_ID, topic, qos, data_len, data);
    return at_send_command(cmd, "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
}

/*---------------------------------------------------------------------------
 Name        : at_mqtt_register_callback
 Input       : mqtt_msg_callback_t callback
 Output      : None
 Description :
 注册一个 MQTT 消息回调，用于接收并处理下行消息（来自 `+MQTTURC: "message"`）。

 输入参数：
 - callback：回调函数指针，签名见 `mqtt_msg_callback_t`。

 行为说明：
 - 将 callback 存入 `s_mqtt_cbs[]` 的空闲槽位（最多 MQTT_CB_MAX 个）；
 - 首次注册时，同时向 AT 解析层注册 URC 关键字：
   - `at_register_urc("+MQTTURC:", mqtt_urc_handler)`
   确保收到 MQTTURC 行时会调用 `mqtt_urc_handler()` 进行分发。

 注意事项：
 - 当前实现不去重：同一个 callback 可被重复注册（会占用多个槽位）；
 - 注册满时静默失败（找不到空槽位就退出），建议上层在需要时扩展返回值/错误提示。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : at_mqtt_unregister_callback
 Input       : mqtt_msg_callback_t callback
 Output      : None
 Description :
 反注册一个 MQTT 消息回调，从 `s_mqtt_cbs[]` 表中移除。

 输入参数：
 - callback：要移除的回调函数指针

 行为说明：
 - 遍历回调表，找到第一个匹配项并清除 used/cb；
 - 当前实现不会在最后一个回调移除时取消 URC 注册（`s_urc_registered` 保持为 true），
   以简化实现并避免频繁增删导致的状态复杂度。
---------------------------------------------------------------------------*/
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