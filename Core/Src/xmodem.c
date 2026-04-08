#include "xmodem.h"
#include "flash_spi.h"
#include "config.h"
#include "uart_at.h"
#include <string.h>

#define SOH 0x01
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CAN 0x18

static xmodem_state_t state = XMODEM_IDLE;
static uint8_t block_buf[XMODEM_BLOCK_SIZE + 4];
static uint8_t block_num = 1;
static uint16_t crc16_table[256];
static uint32_t total_received = 0;
static uint32_t total_length = 0;
static int progress = 0;

static void init_crc16_table(void) {
    for (int i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
        }
        crc16_table[i] = crc;
    }
}

static uint16_t crc16_calc(uint8_t *data, int len) {
    uint16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = (crc << 8) ^ crc16_table[(crc >> 8) ^ data[i]];
    }
    return crc;
}

void xmodem_init(void) {
    init_crc16_table();
    state = XMODEM_IDLE;
}

void xmodem_start(void) {
    block_num = 1;
    total_received = 0;
    progress = 0;
    state = XMODEM_RECEIVING;
}

void xmodem_stop(void) {
    state = XMODEM_IDLE;
}

xmodem_state_t xmodem_get_state(void) {
    return state;
}

int xmodem_get_progress(void) {
    if (total_length == 0) return 0;
    return (total_received * 100) / total_length;
}

void xmodem_process_byte(uint8_t byte) {
    static int index = 0;
    static uint8_t expected_block = 1;

    switch (state) {
    case XMODEM_IDLE:
        if (byte == NAK || byte == 'C') {
            xmodem_start();
            index = 0;
            expected_block = 1;
        }
        break;

    case XMODEM_RECEIVING:
        // 处理EOT（传输结束）
        if (byte == EOT) {
            uint8_t ack = ACK;
            at_send_raw(&ack, 1);
            state = XMODEM_COMPLETE;
            index = 0;
            return;
        }

        // 处理CAN（发送方取消传输）
        if (byte == CAN) {
            state = XMODEM_ERROR;
            index = 0;
            return;
        }

        block_buf[index++] = byte;

        if (index == 133) {
            if (block_buf[1] == expected_block &&
                block_buf[2] == (uint8_t)(~expected_block)) {

                uint16_t crc = (block_buf[131] << 8) | block_buf[132];
                uint16_t calc_crc = crc16_calc(&block_buf[3], 128);

                if (crc == calc_crc) {
                    flash_spi_write(OTA_FLASH_ADDR + (expected_block - 1) * 128,
                                   &block_buf[3], 128);
                    total_received += 128;

                    uint8_t ack = ACK;
                    at_send_raw(&ack, 1);  // 回复ACK给发送端
                    expected_block++;
                    progress = xmodem_get_progress();
                } else {
                    uint8_t nak = NAK;
                    at_send_raw(&nak, 1);  // 回复NAK，发送端将重发当前块
                }
            } else if (block_buf[1] == 0xFF && block_buf[2] == 0x00) {
                // 块号255后归零，继续接收最后一个不满128字节的块
                uint8_t ack = ACK;
                at_send_raw(&ack, 1);
                state = XMODEM_COMPLETE;
            }
            index = 0;
        }
        break;

    case XMODEM_COMPLETE:
    case XMODEM_ERROR:
        break;
    }
}
