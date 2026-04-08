#ifndef MD5_H
#define MD5_H

#include <stdint.h>
#include <stddef.h>

// MD5上下文
typedef struct {
    unsigned int count[2];
    unsigned int state[4];
    unsigned char buffer[64];
} MD5_CTX;

// MD5操作函数
void MD5_Init(MD5_CTX *context);
void MD5_Update(MD5_CTX *context, unsigned char *input, unsigned int inputlen);
void MD5_Final(MD5_CTX *context, unsigned char digest[16]);

// 计算字符串的MD5（返回16字符hex串，取字节4~11）
// 用于计算prov_code: MD5(product_secret + device_sn)
void MD5_Encryption(const char *input, char *result);

#endif // MD5_H