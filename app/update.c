#include <stdint.h>

#include CMSIS_device_header

#include "update.h"
#include "watchdog.h"

static bool do_reset = false;

/* Private typedef -----------------------------------------------------------*/
typedef void (*pFunction)(void);

/* Public functions ----------------------------------------------------------*/
void jmp_btl(void)
{
  /* Jump to system bootloader */
  uint32_t JumpAddress = *(__IO uint32_t *) (BTL_BASE + 4);
  pFunction JumpTo = (pFunction) JumpAddress;

  WDI();
  HAL_DeInit();
  __disable_irq();
  /* Initialize Stack Pointer */
  __set_MSPLIM(0);
  __set_MSP(*(__IO uint32_t *) BTL_BASE);
  JumpTo();
}

void schedule_reset(void)
{
  do_reset = true;
}

bool should_reset(void)
{
  return do_reset;
}
