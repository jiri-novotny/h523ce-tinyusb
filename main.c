#include <stdint.h>

#include CMSIS_device_header

#include "adc.h"
#include "clk.h"
#include "gpio.h"
#include "usb.h"
#include "watchdog.h"

#include "tusb.h"
#include "update.h"
#include "usart.h"

extern void usb_send_button(bool state);
extern void usb_send_uart(void);
extern void usb_send_modem(bool state);

/* Private user code ---------------------------------------------------------*/

void HardFault_Handler(void)
{
  __asm("BKPT #0\n");
}

/* Public user code ---------------------------------------------------------*/
uint32_t modem_state = 0;

int main(void)
{
  uint32_t blink_timer = 0;
  uint32_t button_debounce = 0;
  uint32_t button_state = 0;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  watchdog_init();
  gpio_init();
  adc_init();
  u5_init();
  usb_init();

  uint32_t tick = 0;
  uint32_t tick10ms = HAL_GetTick();
  while (1)
  {
    /* run fast */
    tick = HAL_GetTick();
    tud_task(); // tinyusb device task
    if ((tick - tick10ms) < 10)
    {
      continue;
    }
    tick10ms = tick;

    /* run every 10ms */
    blink_timer++;
    if (blink_timer == 1)
    {
      led_run(true);
    }
    else if (blink_timer == 2)
    {
      led_run(false);
    }
    else if (blink_timer == 100)
    {
      blink_timer = 0;
      WDI();
    }

    if (button_debounce == 14)
    {
      button_debounce++;
      button_state = 1;
      usb_send_button(true);

    }
    else if (button())
    {
      button_debounce++;
    }
    else
    {
      button_debounce = 0;
      if (button_state == 1)
      {
        button_state = 0;
        usb_send_button(false);
      }
    }

    if (adc_complete())
    {
      adc_send();
    }

    if (modem_state == 0 && !modem())
    {
      usb_send_modem(modem_state);
      modem_state = 2;
    }
    else if (modem_state == 1 && modem())
    {
      usb_send_modem(modem_state);
      modem_state = 2;
    }

    if (u5_line_status())
    {
      usb_send_uart();
    }

    if (should_reset())
    {
      NVIC_SystemReset();
    }
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  __disable_irq();
  led_run(true);
  while (1)
  {
  }
}
