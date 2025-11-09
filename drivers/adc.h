#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

void adc_init(void);
void adc_start(void);
bool adc_complete(void);
void adc_send(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
