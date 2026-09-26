#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C"
{
#endif

void u5_init(void);
void u5_write(uint8_t *data, uint32_t len);
uint32_t u5_line_status(void);
int32_t u5_read(void);

#ifdef __cplusplus
}
#endif
#endif /* __USART_H__ */
