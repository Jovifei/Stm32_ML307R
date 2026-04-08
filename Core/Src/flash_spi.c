#include "flash_spi.h"
#include "config.h"

extern SPI_HandleTypeDef FLASH_SPI;

// W25Qxx Flash命令
#define CMD_READ_STATUS    0x05
#define CMD_WRITE_ENABLE   0x06
#define CMD_READ_DATA      0x03
#define CMD_PAGE_PROGRAM   0x02
#define CMD_SECTOR_ERASE   0x20
#define CMD_CHIP_ERASE     0xC7

#define FLASH_CS_PORT  GPIOB
#define FLASH_CS_PIN   GPIO_PIN_14

void flash_spi_init(void) {
    // SPI已在CubeMX配置
    // CS pin需要在GPIO_Init中配置
}

static void flash_cs_low(void) {
    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
}

static void flash_cs_high(void) {
    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);
}

void flash_spi_read(uint32_t addr, uint8_t *data, uint32_t len) {
    uint8_t cmd[4] = {CMD_READ_DATA, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF};

    flash_cs_low();
    HAL_SPI_Transmit(&FLASH_SPI, cmd, 4, 100);
    HAL_SPI_Receive(&FLASH_SPI, data, len, 100);
    flash_cs_high();
}

void flash_spi_write(uint32_t addr, uint8_t *data, uint32_t len) {
    uint8_t enable = CMD_WRITE_ENABLE;

    // 发送写使能
    flash_cs_low();
    HAL_SPI_Transmit(&FLASH_SPI, &enable, 1, 100);
    flash_cs_high();

    // 发送写命令和地址
    uint8_t cmd[4] = {CMD_PAGE_PROGRAM, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF};
    flash_cs_low();
    HAL_SPI_Transmit(&FLASH_SPI, cmd, 4, 100);
    HAL_SPI_Transmit(&FLASH_SPI, data, len, 100);
    flash_cs_high();
}

void flash_spi_erase(uint32_t addr, uint32_t len) {
    uint8_t enable = CMD_WRITE_ENABLE;
    uint8_t cmd[4] = {CMD_SECTOR_ERASE, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF};

    flash_cs_low();
    HAL_SPI_Transmit(&FLASH_SPI, &enable, 1, 100);
    flash_cs_high();

    flash_cs_low();
    HAL_SPI_Transmit(&FLASH_SPI, cmd, 4, 100);
    flash_cs_high();

    HAL_Delay(400);
}
