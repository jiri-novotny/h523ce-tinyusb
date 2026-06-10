#include <stdint.h>

#include CMSIS_device_header

#include "update.h"

static bool do_reset = false;

/* Private typedef -----------------------------------------------------------*/
typedef void (*pFunction)(void);

/* Private define ------------------------------------------------------------*/
#define BTL_BASE    0x0BF97000U

/* Public functions ----------------------------------------------------------*/
void jmp_btl(void)
{
  /* Jump to user application */
  uint32_t JumpAddress = *(__IO uint32_t *) (BTL_BASE + 4);
  pFunction JumpToApplication = (pFunction) JumpAddress;

  __disable_irq();
  /* Initialize user application's Stack Pointer */
  __set_MSP(*(__IO uint32_t *) BTL_BASE);
  JumpToApplication();
}

void schedule_reset(void)
{
  do_reset = true;
}

bool should_reset(void)
{
  return do_reset;
}
