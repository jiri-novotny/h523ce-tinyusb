#include <stdint.h>

#include CMSIS_device_header
#include "tusb.h"

#include "adc.h"
#include "clk.h"
#include "gpio.h"
#include "watchdog.h"

#include "update.h"

/* Private user code ---------------------------------------------------------*/

/* Public user code ---------------------------------------------------------*/

int main(void)
{
  uint32_t blink_timer = 0;
  uint32_t button_debounce = 0;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  gpio_init();
  adc_init();
//  usb_start();
  watchdog_init();

#if 0
  uint32_t tick = 0;
  uint32_t tick10ms = HAL_GetTick();
  while (1)
  {
    /* run fast */
    tick = HAL_GetTick();
    usb_receive();
    if ((tick - tick10ms) < 10)
    {
      continue;
    }
    tick10ms = tick;
#else
  while (1)
  {
      HAL_Delay(10);
#endif
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
      // button action
      led_run(true);
    }
    else if (button())
    {
      button_debounce++;
    }
    else
    {
      button_debounce = 0;
    }

    if (adc_complete())
    {
      adc_send();
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
