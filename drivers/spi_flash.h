#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* GD25Q128 - 128 Mbit (16 MiB) SPI NOR flash */
#define SPI_FLASH_PAGE_SIZE     256U
#define SPI_FLASH_SECTOR_SIZE   4096U
#define SPI_FLASH_SIZE          (16U * 1024U * 1024U)
#define SPI_FLASH_SECTOR_COUNT  (SPI_FLASH_SIZE / SPI_FLASH_SECTOR_SIZE)

/* return codes */
#define SPI_FLASH_OK            0
#define SPI_FLASH_ERR_ID        -1
#define SPI_FLASH_ERR_TIMEOUT   -2
#define SPI_FLASH_ERR_RANGE     -3

int spi_flash_init(void);
uint32_t spi_flash_jedec_id(void);
int spi_flash_read(uint32_t addr, uint8_t *data, uint32_t len);
int spi_flash_write(uint32_t addr, const uint8_t *data, uint32_t len);
int spi_flash_erase_sector(uint32_t addr);
int spi_flash_erase_chip(void);

#ifdef __cplusplus
}
#endif
#endif /* __SPI_FLASH_H__ */
