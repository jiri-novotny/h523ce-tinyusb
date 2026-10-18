#ifndef __FS_CMD_H__
#define __FS_CMD_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* HID protocol headers, see tusb_config.c */
#define HDR_FS_OPEN    6
#define HDR_FS_WRITE   7
#define HDR_FS_READ    8
#define HDR_FS_CLOSE   9
#define HDR_FS_BENCH   10
#define HDR_FS_INFO    11

/* HID OUT handlers, called from tud_hid_set_report_cb() */
void fs_cmd_open(const uint8_t *data, uint8_t len);
void fs_cmd_write(const uint8_t *data, uint8_t len);
void fs_cmd_read(void);
void fs_cmd_close(void);
void fs_cmd_bench(const uint8_t *data, uint8_t len);
void fs_cmd_info(void);

/* call from main loop - sends replies, streams file reads, runs benchmark */
void fs_cmd_task(void);

#ifdef __cplusplus
}
#endif
#endif /* __FS_CMD_H__ */
