#include <stdbool.h>
#include <stdint.h>

#include CMSIS_device_header

#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_spi.h"

#include "spi_flash.h"

/*
 * GD25Q128 on SPI1 (see WeAct STM32H523CxTx schematic, U3)
 *   PA4 - F_CS (GPIO)
 *   PA5 - SCK  (AF5)
 *   PA6 - MISO (AF5)
 *   PA7 - MOSI (AF5)
 * WP and HOLD are tied high on the board.
 */
#define FLASH_SPI           SPI1
#define FLASH_CS_PORT       GPIOA
#define FLASH_CS_PIN        LL_GPIO_PIN_4

#define CMD_WRITE_ENABLE    0x06
#define CMD_READ_SR1        0x05
#define CMD_WRITE_SR1       0x01
#define CMD_READ_DATA       0x03
#define CMD_PAGE_PROGRAM    0x02
#define CMD_SECTOR_ERASE    0x20
#define CMD_CHIP_ERASE      0xC7
#define CMD_JEDEC_ID        0x9F
#define CMD_RELEASE_PD      0xAB
#define CMD_RESET_ENABLE    0x66
#define CMD_RESET           0x99

#define SR1_WIP             0x01
#define SR1_BP_MASK         0x7C /* BP0..BP4 */

#define JEDEC_MFR_GIGADEVICE    0xC8

/* max times from GD25Q128E datasheet, with margin */
#define TIMEOUT_PROGRAM_MS  10
#define TIMEOUT_SR_MS       50
#define TIMEOUT_SECTOR_MS   500
#define TIMEOUT_CHIP_MS     200000

static uint32_t flash_id = 0;

static inline void cs_low(void)
{
    LL_GPIO_ResetOutputPin(FLASH_CS_PORT, FLASH_CS_PIN);
}

static inline void cs_high(void)
{
    LL_GPIO_SetOutputPin(FLASH_CS_PORT, FLASH_CS_PIN);
}

static uint8_t spi_xfer_byte(uint8_t tx)
{
    while (!LL_SPI_IsActiveFlag_TXP(FLASH_SPI))
    {
    }
    LL_SPI_TransmitData8(FLASH_SPI, tx);
    while (!LL_SPI_IsActiveFlag_RXP(FLASH_SPI))
    {
    }
    return LL_SPI_ReceiveData8(FLASH_SPI);
}

static void spi_tx(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        spi_xfer_byte(data[i]);
    }
}

static void spi_rx(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        data[i] = spi_xfer_byte(0xFF);
    }
}

static void cmd_simple(uint8_t cmd)
{
    cs_low();
    spi_xfer_byte(cmd);
    cs_high();
}

static void cmd_addr(uint8_t cmd, uint32_t addr)
{
    uint8_t buf[4] = {cmd, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
    spi_tx(buf, sizeof(buf));
}

static uint8_t read_sr1(void)
{
    uint8_t sr;

    cs_low();
    spi_xfer_byte(CMD_READ_SR1);
    sr = spi_xfer_byte(0xFF);
    cs_high();

    return sr;
}

static int wait_ready(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (read_sr1() & SR1_WIP)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            return SPI_FLASH_ERR_TIMEOUT;
        }
    }
    return SPI_FLASH_OK;
}

static void spi_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_SPI_InitTypeDef SPI_InitStruct = {0};

    /* PLL1Q = 12.8 MHz (see clk.c), SCK = PLL1Q / 2 */
    LL_RCC_SetSPIClockSource(LL_RCC_SPI1_CLKSOURCE_PLL1Q);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    /* CS - output, idle high */
    cs_high();
    GPIO_InitStruct.Pin = FLASH_CS_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(FLASH_CS_PORT, &GPIO_InitStruct);

    /* SCK, MISO, MOSI */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* mode 0, 8 bit, MSB first, software NSS */
    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 0x7;
    LL_SPI_Init(FLASH_SPI, &SPI_InitStruct);

    LL_SPI_SetInternalSSLevel(FLASH_SPI, LL_SPI_SS_LEVEL_HIGH);
    LL_SPI_SetFIFOThreshold(FLASH_SPI, LL_SPI_FIFO_TH_01DATA);
    LL_SPI_EnableGPIOControl(FLASH_SPI);

    /* endless transfer - master clocks only while TX FIFO holds data */
    LL_SPI_SetTransferSize(FLASH_SPI, 0);
    LL_SPI_Enable(FLASH_SPI);
    LL_SPI_StartMasterTransfer(FLASH_SPI);
}

int spi_flash_init(void)
{
    uint8_t id[3];

    spi_init();

    /* wake up from deep power-down and reset, in case MCU reset hit mid-operation */
    cmd_simple(CMD_RELEASE_PD);
    HAL_Delay(1);
    cmd_simple(CMD_RESET_ENABLE);
    cmd_simple(CMD_RESET);
    HAL_Delay(1);

    cs_low();
    spi_xfer_byte(CMD_JEDEC_ID);
    spi_rx(id, sizeof(id));
    cs_high();
    flash_id = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];

    /* expect 0xC84018; accept other vendors with the same capacity (e.g. W25Q128 0xEF4018) */
    if (id[0] == 0x00 || id[0] == 0xFF || id[2] != 0x18)
    {
        return SPI_FLASH_ERR_ID;
    }

    /* clear block protection, if any */
    if (read_sr1() & SR1_BP_MASK)
    {
        cmd_simple(CMD_WRITE_ENABLE);
        cs_low();
        spi_xfer_byte(CMD_WRITE_SR1);
        spi_xfer_byte(0x00);
        cs_high();
        if (wait_ready(TIMEOUT_SR_MS) != SPI_FLASH_OK)
        {
            return SPI_FLASH_ERR_TIMEOUT;
        }
    }

    return SPI_FLASH_OK;
}

uint32_t spi_flash_jedec_id(void)
{
    return flash_id;
}

int spi_flash_read(uint32_t addr, uint8_t *data, uint32_t len)
{
    if (addr + len > SPI_FLASH_SIZE)
    {
        return SPI_FLASH_ERR_RANGE;
    }

    cs_low();
    cmd_addr(CMD_READ_DATA, addr);
    spi_rx(data, len);
    cs_high();

    return SPI_FLASH_OK;
}

int spi_flash_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if (addr + len > SPI_FLASH_SIZE)
    {
        return SPI_FLASH_ERR_RANGE;
    }

    while (len > 0)
    {
        /* page program must not cross a page boundary */
        uint32_t chunk = SPI_FLASH_PAGE_SIZE - (addr % SPI_FLASH_PAGE_SIZE);
        if (chunk > len)
        {
            chunk = len;
        }

        cmd_simple(CMD_WRITE_ENABLE);
        cs_low();
        cmd_addr(CMD_PAGE_PROGRAM, addr);
        spi_tx(data, chunk);
        cs_high();

        if (wait_ready(TIMEOUT_PROGRAM_MS) != SPI_FLASH_OK)
        {
            return SPI_FLASH_ERR_TIMEOUT;
        }

        addr += chunk;
        data += chunk;
        len -= chunk;
    }

    return SPI_FLASH_OK;
}

int spi_flash_erase_sector(uint32_t addr)
{
    if (addr >= SPI_FLASH_SIZE)
    {
        return SPI_FLASH_ERR_RANGE;
    }

    cmd_simple(CMD_WRITE_ENABLE);
    cs_low();
    cmd_addr(CMD_SECTOR_ERASE, addr & ~(SPI_FLASH_SECTOR_SIZE - 1));
    cs_high();

    return wait_ready(TIMEOUT_SECTOR_MS);
}

int spi_flash_erase_chip(void)
{
    cmd_simple(CMD_WRITE_ENABLE);
    cmd_simple(CMD_CHIP_ERASE);

    return wait_ready(TIMEOUT_CHIP_MS);
}
