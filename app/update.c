#include <stdint.h>

#include CMSIS_device_header

#include "update.h"
#include "watchdog.h"

#define BTL_MAGIC 0xB00710ADU

static bool do_reset = false;

/* survives NVIC_SystemReset(), see .noinit in linker script */
static uint32_t btl_request __attribute__((section(".noinit")));

/* Private typedef -----------------------------------------------------------*/
typedef void (*pFunction)(void);

/* Public functions ----------------------------------------------------------*/

/*
 * Request jump to system bootloader. Jumping from the running application leaves
 * enabled/pending interrupts (EXTI14, USB, DMA, ...) and IWDG behind, the bootloader
 * then hangs in its default handler. Instead reset the MCU and jump from btl_check()
 * with reset state.
 */
void jmp_btl(void)
{
  btl_request = BTL_MAGIC;
  schedule_reset();
}

/* must be called first in main(), before any clock or peripheral init */
void btl_check(void)
{
  if (btl_request != BTL_MAGIC)
  {
    return;
  }
  btl_request = 0;

  uint32_t JumpAddress = *(__IO uint32_t *) (BTL_BASE + 4);
  pFunction JumpTo = (pFunction) JumpAddress;

  SCB->VTOR = BTL_BASE;
  __set_MSPLIM(0);
  __set_PSPLIM(0);
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
