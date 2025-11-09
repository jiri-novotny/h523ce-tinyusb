#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

void gpio_init(void);

/* inputs */
bool button(void);

/* outputs */
void led_run(bool on);

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */
