#ifndef __FS_H__
#define __FS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "lfs.h"

/* littlefs instance on external SPI flash, valid after fs_init() returns 0 */
extern lfs_t fs_lfs;
extern bool fs_mounted;

int fs_init(void);

#ifdef __cplusplus
}
#endif
#endif /* __FS_H__ */
