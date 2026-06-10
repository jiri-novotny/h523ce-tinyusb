#ifndef UPDATE_H_DEFINED
#define UPDATE_H_DEFINED

#include <stdint.h>
#include <stdbool.h>

void jmp_btl(void);
void schedule_reset(void);
bool should_reset(void);

#endif /* UPDATE_H_DEFINED */
