#include "at_config.h"
#include <string.h>

// ==================== EMQ服务器CA证书 ====================
// 来源: D:\work\ESP32_CT\main\iot\emq.c - emq_ca_cert
// 用于验证MQTT服务器证书（单向认证时使用）
const char g_mqtt_ca_cert[] =
"-----BEGIN CERTIFICATE-----\n"
"MIIDSTCCAjGgAwIBAgIUGDm+nCjNDkxdFsPoXpEhEZcp29YwDQYJKoZIhvcNAQEL\n"
"BQAwNDELMAkGA1UEBhMCQ04xCzAJBgNVBAgMAlpKMQswCQYDVQQHDAJIWjELMAkG\n"
"A1UECgwCRE0wHhcNMjAwODE5MTAzNjI0WhcNMzAwODE3MTAzNjI0WjA0MQswCQYD\n"
"VQQGEwJDTjELMAkGA1UECAwCWkoxCzAJBgNVBAcMAkhaMQswCQYDVQQKDAJETTCC\n"
"ASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBANsGzJ8WLtQo2XCjA+t4NrFx\n"
"gZ9+PCK5Bcvq63ZgRz7uBiS0fDw+9qpOL4SvPmYIAjTj4dlmrk7g4UWk4gHXHxpE\n"
"Ia0FX1UorCpDOmcW4PFIqWqgak586ZAW2WkJUJSpzn2qRc7NFKONB9i1Q3F8Fztl\n"
"x2n9m2KMkOtCe5pKoiFl34aGdcCphC7nRvaj+EHM85ZbvTEyaLW/J2TFleofUbS0\n"
"jNzvPNnkSrol1MI/4RmabY61wI5IT/fr3ADRnwh8GTSm1COe9Cio3Rhes06GWjns\n"
"Mrivsf/9KMPdl5YLnnRl8pDB6qYF1Fg7cnj7UmAyfsn44I+GyOGnHisT2OykrYsC\n"
"AwEAAaNTMFEwHQYDVR0OBBYEFJ4w7L1q1AjdnMcPsJLZIF0ily4JMB8GA1UdIwQY\n"
"MBaAFJ4w7L1q1AjdnMcPsJLZIF0ily4JMA8GA1UdEwEB/wQFMAMBAf8wDQYJKoZI\n"
"hvcNAQELBQADggEBACWwWG0RkCOlTjzUbZ9yoWBdG/Pq0OrvtNjPc3klSrWsMXr9\n"
"5QL9Gd8ZAj2DKEpOcWA99KFxwHj0e5az0/9WL5If1tIt+itHKbzmTKHIZlvTmJXj\n"
"U9dj4Ni0JxBW+Y+I2HxfQJQHbBRwIbdn0xIIe2wngzcaGhDgrXpUBMAix/ZI9C6i\n"
"pxsEqnv8Y+380bszVVspNmePMbdjMdgDVgAYYm6M1t7AsDuC0Yc5H7CWW2c9Dxj/\n"
"x5+/EfAfRODf06WIloS4cVdPaJSVtVnSRo4vZIwNA0Ejbq8QJv/Z0grBwmjbu+La\n"
"1CJZrUyESMdK9axtXhhk7k0GCEtptBc0qaEvLys=\n"
"-----END CERTIFICATE-----";

const uint32_t g_mqtt_ca_cert_len = sizeof(g_mqtt_ca_cert) - 1;

// ==================== 客户端证书（需从服务器获取或预配置）====================
// 客户端证书由服务器动态颁发，此处为占位符
// 实际使用时需要通过HTTPS从服务器获取并存储到Flash
const char g_mqtt_client_cert[] = "";

const uint32_t g_mqtt_client_cert_len = 0;

// ==================== 客户端私钥（需从服务器获取或预配置）====================
// 客户端私钥由服务器动态颁发，此处为占位符
// 实际使用时需要通过HTTPS从服务器获取并存储到Flash
const char g_mqtt_client_key[] = "";

const uint32_t g_mqtt_client_key_len = 0;