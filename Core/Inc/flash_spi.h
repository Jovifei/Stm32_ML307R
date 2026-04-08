#ifndef FLASH_SPI_H
#define FLASH_SPI_H

#include "stdint.h"

// 初始化Flash
void flash_spi_init(void);

// 读取数据
void flash_spi_read(uint32_t addr, uint8_t *data, uint32_t len);

// 写入数据
void flash_spi_write(uint32_t addr, uint8_t *data, uint32_t len);

// 擦除扇区
void flash_spi_erase(uint32_t addr, uint32_t len);

#endif // FLASH_SPI_H
