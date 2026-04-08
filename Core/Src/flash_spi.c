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

/*---------------------------------------------------------------------------
 Name        : flash_spi_init
 Input       : None
 Output      : None
 Description :
 初始化外部 SPI Flash 访问模块的软硬件依赖。

 设计说明：
 - 本模块默认外设 SPI 由 CubeMX/HAL 完成初始化（例如 SPI 时钟、模式、分频等），
   因此此处不重复初始化 `FLASH_SPI`。
 - 片选 CS 引脚需要在 GPIO 初始化中配置为推挽输出，并保持空闲时为高电平（不选中）。

 典型使用时机：
 - 系统启动后、开始读写外部 Flash（存储设备凭证/证书/OTA 包等）之前调用一次。

 注意：
 - 当前实现为空壳，主要作为模块化接口保留；如需在此处做自检，可扩展读取 JEDEC ID、
   检查写使能/忙状态等。
---------------------------------------------------------------------------*/
void flash_spi_init(void) {
    // SPI已在CubeMX配置
    // CS pin需要在GPIO_Init中配置
}

/*---------------------------------------------------------------------------
 Name        : flash_cs_low
 Input       : None
 Output      : None
 Description :
 拉低外部 Flash 的片选信号（CS=0），开始一次 SPI 事务。

 约束：
 - 该函数假设 CS 引脚已正确配置为输出模式；
 - 与 `flash_cs_high()` 成对使用，确保一次命令/数据传输期间 CS 保持有效。
---------------------------------------------------------------------------*/
static void flash_cs_low(void) {
    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
}

/*---------------------------------------------------------------------------
 Name        : flash_cs_high
 Input       : None
 Output      : None
 Description :
 拉高外部 Flash 的片选信号（CS=1），结束一次 SPI 事务。

 约束：
 - 必须在完成命令+地址+数据的传输后再拉高；
 - 多数 Flash 器件以 CS 上升沿作为命令帧结束标志。
---------------------------------------------------------------------------*/
static void flash_cs_high(void) {
    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);
}

/*---------------------------------------------------------------------------
 Name        : flash_spi_read
 Input       : uint32_t addr, uint8_t *data, uint32_t len
 Output      : None
 Description :
 从外部 SPI Flash 读取连续数据到 RAM 缓冲区。

 输入参数：
 - addr：Flash 线性地址（24-bit 地址帧：A23..A0），当前实现直接发送 3 字节地址。
 - data：输出缓冲区指针，用于接收读取的数据。
 - len：要读取的字节数。

 通信序列（当前实现）：
 - CS 拉低
 - 发送 READ 命令 `CMD_READ_DATA` (0x03) + 3 字节地址
 - 继续时钟并接收 len 字节数据
 - CS 拉高

 注意事项：
 - 未对参数合法性（data==NULL、len==0）做检查，调用方需保证输入有效；
 - 未处理跨页/对齐限制（READ 命令通常不限制），但不同器件可能有高速读命令要求 dummy cycles；
 - 超时参数当前固定 100ms，若读取较大块数据建议调整或改用 DMA。
---------------------------------------------------------------------------*/
void flash_spi_read(uint32_t addr, uint8_t *data, uint32_t len) {
    uint8_t cmd[4] = {CMD_READ_DATA, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF};

    flash_cs_low();
    HAL_SPI_Transmit(&FLASH_SPI, cmd, 4, 100);
    HAL_SPI_Receive(&FLASH_SPI, data, len, 100);
    flash_cs_high();
}

/*---------------------------------------------------------------------------
 Name        : flash_spi_write
 Input       : uint32_t addr, uint8_t *data, uint32_t len
 Output      : None
 Description :
 向外部 SPI Flash 写入连续数据（页编程 Page Program）。

 输入参数：
 - addr：Flash 线性地址（24-bit），写入起始地址。
 - data：待写入数据指针。
 - len：待写入字节数。

 通信序列（当前实现）：
 - 发送写使能 `CMD_WRITE_ENABLE` (0x06)
 - 发送页编程命令 `CMD_PAGE_PROGRAM` (0x02) + 3 字节地址
 - 发送 len 字节数据

 重要限制（当前实现未处理）：
 - Page Program 通常受“页边界”限制（例如 256B 页），跨页写入需要拆分；
 - 写入后必须轮询 WIP（写入进行中）位或等待足够时间，保证内部编程完成；
 - 写前若目标区域非 0xFF，需先擦除（sector erase）。

 当前实现的行为风险：
 - 未轮询忙状态/未等待写完成，若紧接着读或再次写，可能得到不一致结果；
 - 未拆分跨页写入，若 len 跨页，实际写入可能回绕覆盖页首。
---------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------
 Name        : flash_spi_erase
 Input       : uint32_t addr, uint32_t len
 Output      : None
 Description :
 擦除外部 SPI Flash 指定区域（按扇区擦除命令执行）。

 输入参数：
 - addr：擦除起始地址（通常需要按扇区大小对齐，例如 4KB 对齐）。
 - len：擦除长度（当前实现未使用 len 做循环分段，仅执行一次扇区擦除）。

 通信序列（当前实现）：
 - 发送写使能 `CMD_WRITE_ENABLE` (0x06)
 - 发送扇区擦除 `CMD_SECTOR_ERASE` (0x20) + 3 字节地址
 - 固定延时 `HAL_Delay(400)` 等待擦除完成（经验值）

 注意事项：
 - 当前实现只擦除一个扇区，并未根据 len 进行多扇区循环擦除；
 - 仅靠固定延时不可靠：不同温度/电压/器件型号擦除时间差异大，建议改为轮询状态寄存器 WIP；
 - addr 若未对齐扇区边界，器件会按内部规则向下取整到扇区起始或行为未定义（视型号而定）。
---------------------------------------------------------------------------*/
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
