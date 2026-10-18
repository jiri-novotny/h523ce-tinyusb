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
bool modem(void);

/* outputs */
void led_run(bool on);
void led_toggle(void);

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */
