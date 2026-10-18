#include <stdbool.h>
#include <stdint.h>

#include CMSIS_device_header

#include "gpio.h"
#include "lfs.h"
#include "spi_flash.h"
#include "watchdog.h"

#include "fs.h"

#define FS_READ_SIZE        16
#define FS_PROG_SIZE        16
#define FS_CACHE_SIZE       SPI_FLASH_PAGE_SIZE
#define FS_LOOKAHEAD_SIZE   128 /* bytes, 8 blocks per byte */
#define FS_BLOCK_CYCLES     500

lfs_t fs_lfs;
bool fs_mounted = false;

static uint8_t fs_read_buffer[FS_CACHE_SIZE] __attribute__((aligned(4)));
static uint8_t fs_prog_buffer[FS_CACHE_SIZE] __attribute__((aligned(4)));
static uint8_t fs_lookahead_buffer[FS_LOOKAHEAD_SIZE] __attribute__((aligned(8)));

static int bd_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    uint32_t addr = block * c->block_size + off;
    return spi_flash_read(addr, buffer, size) == SPI_FLASH_OK ? LFS_ERR_OK : LFS_ERR_IO;
}

static int bd_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer,
                   lfs_size_t size)
{
    uint32_t addr = block * c->block_size + off;
    return spi_flash_write(addr, buffer, size) == SPI_FLASH_OK ? LFS_ERR_OK : LFS_ERR_IO;
}

static int bd_erase(const struct lfs_config *c, lfs_block_t block)
{
    return spi_flash_erase_sector(block * c->block_size) == SPI_FLASH_OK ? LFS_ERR_OK : LFS_ERR_IO;
}

static int bd_sync(const struct lfs_config *c)
{
    /* program/erase already wait for completion */
    return LFS_ERR_OK;
}

static const struct lfs_config fs_cfg = {
    .read = bd_read,
    .prog = bd_prog,
    .erase = bd_erase,
    .sync = bd_sync,

    .read_size = FS_READ_SIZE,
    .prog_size = FS_PROG_SIZE,
    .block_size = SPI_FLASH_SECTOR_SIZE,
    .block_count = SPI_FLASH_SECTOR_COUNT,
    .block_cycles = FS_BLOCK_CYCLES,
    .cache_size = FS_CACHE_SIZE,
    .lookahead_size = FS_LOOKAHEAD_SIZE,

    .read_buffer = fs_read_buffer,
    .prog_buffer = fs_prog_buffer,
    .lookahead_buffer = fs_lookahead_buffer,
};

int fs_init(void)
{
    int err;

    if (spi_flash_init() != SPI_FLASH_OK)
    {
        return LFS_ERR_IO;
    }

    err = lfs_mount(&fs_lfs, &fs_cfg);
    if (err)
    {
        /* first boot or corrupted - format only on user request (PA0 button),
           otherwise stay here and blink fast to signal the error */
        uint32_t blink = HAL_GetTick();
        while (!button())
        {
            if ((HAL_GetTick() - blink) >= 100)
            {
                blink = HAL_GetTick();
                led_toggle();
            }
            WDI();
        }
        led_run(false);

        err = lfs_format(&fs_lfs, &fs_cfg);
        if (err)
        {
            return err;
        }
        err = lfs_mount(&fs_lfs, &fs_cfg);
    }

    fs_mounted = (err == LFS_ERR_OK);
    return err;
}
