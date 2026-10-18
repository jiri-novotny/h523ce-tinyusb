#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include CMSIS_device_header

#include "fs.h"
#include "fs_cmd.h"
#include "spi_flash.h"
#include "tusb.h"
#include "watchdog.h"

/*
 * Flash file access / benchmark over HID, packet = HDR, LEN, DATA[LEN], all values little endian
 *
 * host -> device                          device -> host
 * OPEN  mode(u8: 0 rd, 1 wr), name[]      OPEN  status(i8), size(u32)
 * WRITE data[1..62]                       -
 * READ  -                                 READ  data[1..62] ..., then READ with LEN 0 (EOF/error)
 * CLOSE -                                 CLOSE status(i8), bytes(u32)
 * BENCH size(u32), chunk(u16)             BENCH status(i8), write_ms(u32), read_ms(u32), errors(u32)
 * INFO  -                                 INFO  status(i8), jedec(u32), block_size(u32),
 *                                               block_count(u32), used_blocks(u32)
 *
 * status is 0 or a negative LFS_ERR_xxx code; errors during WRITE/READ are reported by CLOSE.
 */

#define PKT_DATA_MAX    (CFG_TUD_HID_EP_BUFSIZE - 2)
#define BENCH_FILE      "bench.bin"
#define BENCH_CHUNK_MAX 4096

static lfs_file_t file;
static struct lfs_file_config file_cfg;
static uint8_t file_buffer[SPI_FLASH_PAGE_SIZE] __attribute__((aligned(4)));
static bool file_open = false;
static bool file_streaming = false;
static int file_err = LFS_ERR_OK;
static uint32_t file_bytes = 0;

static uint8_t reply[CFG_TUD_HID_EP_BUFSIZE];
static uint8_t reply_len = 0;

static bool bench_pending = false;
static uint32_t bench_size;
static uint32_t bench_chunk;
static uint8_t bench_buffer[BENCH_CHUNK_MAX] __attribute__((aligned(4)));

static void put_u32(uint8_t *p, uint32_t v)
{
    memcpy(p, &v, sizeof(v));
}

static uint32_t get_u32(const uint8_t *p)
{
    uint32_t v;
    memcpy(&v, p, sizeof(v));
    return v;
}

/* single pending reply, sent from fs_cmd_task() once IN endpoint is free */
static uint8_t *reply_prepare(uint8_t hdr, uint8_t len, int status)
{
    reply[0] = hdr;
    reply[1] = len;
    reply[2] = (uint8_t)(int8_t)status;
    reply_len = len + 2;
    return &reply[3];
}

static void close_file(void)
{
    if (file_open)
    {
        int err = lfs_file_close(&fs_lfs, &file);
        if (file_err == LFS_ERR_OK)
        {
            file_err = err;
        }
    }
    file_open = false;
    file_streaming = false;
}

void fs_cmd_open(const uint8_t *data, uint8_t len)
{
    char name[LFS_NAME_MAX + 1];
    int flags;
    int err;
    uint8_t *p;

    close_file();
    file_err = LFS_ERR_OK;
    file_bytes = 0;

    if (!fs_mounted)
    {
        err = LFS_ERR_IO;
    }
    else if (len < 2 || (len - 1) > LFS_NAME_MAX)
    {
        err = LFS_ERR_INVAL;
    }
    else
    {
        memcpy(name, &data[1], len - 1);
        name[len - 1] = '\0';
        flags = data[0] ? (LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) : LFS_O_RDONLY;

        memset(&file_cfg, 0, sizeof(file_cfg));
        file_cfg.buffer = file_buffer;
        err = lfs_file_opencfg(&fs_lfs, &file, name, flags, &file_cfg);
        file_open = (err == LFS_ERR_OK);
    }

    p = reply_prepare(HDR_FS_OPEN, 5, err);
    put_u32(p, file_open ? (uint32_t)lfs_file_size(&fs_lfs, &file) : 0);
}

void fs_cmd_write(const uint8_t *data, uint8_t len)
{
    if (!file_open || file_err != LFS_ERR_OK)
    {
        return;
    }

    lfs_ssize_t res = lfs_file_write(&fs_lfs, &file, data, len);
    if (res < 0)
    {
        file_err = res;
    }
    else
    {
        file_bytes += res;
    }
}

void fs_cmd_read(void)
{
    file_streaming = true;
}

void fs_cmd_close(void)
{
    uint8_t *p;

    close_file();
    p = reply_prepare(HDR_FS_CLOSE, 5, file_err);
    put_u32(p, file_bytes);
}

void fs_cmd_bench(const uint8_t *data, uint8_t len)
{
    if (len < 6)
    {
        reply_prepare(HDR_FS_BENCH, 13, LFS_ERR_INVAL);
        memset(&reply[3], 0, 12);
        return;
    }

    bench_size = get_u32(&data[0]);
    bench_chunk = data[4] | ((uint32_t)data[5] << 8);
    if (bench_chunk == 0 || bench_chunk > BENCH_CHUNK_MAX)
    {
        bench_chunk = BENCH_CHUNK_MAX;
    }
    /* run from fs_cmd_task(), not from USB callback */
    bench_pending = true;
}

void fs_cmd_info(void)
{
    lfs_ssize_t used = fs_mounted ? lfs_fs_size(&fs_lfs) : LFS_ERR_IO;
    uint8_t *p = reply_prepare(HDR_FS_INFO, 17, used < 0 ? used : LFS_ERR_OK);

    put_u32(&p[0], spi_flash_jedec_id());
    put_u32(&p[4], SPI_FLASH_SECTOR_SIZE);
    put_u32(&p[8], SPI_FLASH_SECTOR_COUNT);
    put_u32(&p[12], used < 0 ? 0 : (uint32_t)used);
}

/* xorshift32 - same sequence for write and verify */
static void bench_fill(uint32_t *seed, uint8_t *buf, uint32_t len)
{
    uint32_t x = *seed;
    for (uint32_t i = 0; i < len; i++)
    {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        buf[i] = (uint8_t)x;
    }
    *seed = x;
}

static uint32_t bench_verify(uint32_t *seed, const uint8_t *buf, uint32_t len)
{
    uint32_t errors = 0;
    uint32_t x = *seed;
    for (uint32_t i = 0; i < len; i++)
    {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        if (buf[i] != (uint8_t)x)
        {
            errors++;
        }
    }
    *seed = x;
    return errors;
}

static void bench_run(void)
{
    lfs_file_t bf;
    struct lfs_file_config bcfg = {.buffer = file_buffer};
    uint32_t seed;
    uint32_t start;
    uint32_t write_ms = 0;
    uint32_t read_ms = 0;
    uint32_t errors = 0;
    uint32_t done;
    int err;

    /* shares file_buffer with USB file access */
    close_file();

    if (!fs_mounted)
    {
        err = LFS_ERR_IO;
        goto out;
    }

    /* write, pattern generation time excluded */
    seed = 0x12345678;
    start = HAL_GetTick();
    err = lfs_file_opencfg(&fs_lfs, &bf, BENCH_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, &bcfg);
    write_ms += HAL_GetTick() - start;
    if (err)
    {
        goto out;
    }
    for (done = 0; done < bench_size && err == LFS_ERR_OK; done += bench_chunk)
    {
        uint32_t n = (bench_size - done < bench_chunk) ? (bench_size - done) : bench_chunk;
        bench_fill(&seed, bench_buffer, n);
        start = HAL_GetTick();
        lfs_ssize_t res = lfs_file_write(&fs_lfs, &bf, bench_buffer, n);
        write_ms += HAL_GetTick() - start;
        err = (res < 0) ? res : LFS_ERR_OK;
        WDI();
    }
    start = HAL_GetTick();
    int cerr = lfs_file_close(&fs_lfs, &bf);
    write_ms += HAL_GetTick() - start;
    err = err ? err : cerr;
    if (err)
    {
        goto out;
    }

    /* read + verify, verification time excluded */
    seed = 0x12345678;
    err = lfs_file_opencfg(&fs_lfs, &bf, BENCH_FILE, LFS_O_RDONLY, &bcfg);
    if (err)
    {
        goto out;
    }
    for (done = 0; done < bench_size && err == LFS_ERR_OK; done += bench_chunk)
    {
        uint32_t n = (bench_size - done < bench_chunk) ? (bench_size - done) : bench_chunk;
        start = HAL_GetTick();
        lfs_ssize_t res = lfs_file_read(&fs_lfs, &bf, bench_buffer, n);
        read_ms += HAL_GetTick() - start;
        if (res < 0)
        {
            err = res;
        }
        else
        {
            errors += bench_verify(&seed, bench_buffer, res) + (n - res);
        }
        WDI();
    }
    lfs_file_close(&fs_lfs, &bf);

out:
    {
        uint8_t *p = reply_prepare(HDR_FS_BENCH, 13, err);
        put_u32(&p[0], write_ms);
        put_u32(&p[4], read_ms);
        put_u32(&p[8], errors);
    }
}

void fs_cmd_task(void)
{
    if (bench_pending)
    {
        bench_pending = false;
        bench_run();
    }

    if (!tud_hid_ready())
    {
        return;
    }

    if (reply_len)
    {
        if (tud_hid_report(0, reply, reply_len))
        {
            reply_len = 0;
        }
        return;
    }

    if (file_streaming)
    {
        uint8_t pkt[CFG_TUD_HID_EP_BUFSIZE];
        lfs_ssize_t res = (file_open && file_err == LFS_ERR_OK)
                              ? lfs_file_read(&fs_lfs, &file, &pkt[2], PKT_DATA_MAX)
                              : 0;
        if (res < 0)
        {
            file_err = res;
            res = 0;
        }
        pkt[0] = HDR_FS_READ;
        pkt[1] = (uint8_t)res;
        if (tud_hid_report(0, pkt, 2 + res))
        {
            file_bytes += res;
            if (res == 0)
            {
                file_streaming = false; /* EOF sent */
            }
        }
        else if (res > 0)
        {
            /* not sent - rewind so the chunk goes out next time */
            lfs_file_seek(&fs_lfs, &file, -res, LFS_SEEK_CUR);
        }
    }
}
