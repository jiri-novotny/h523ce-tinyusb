/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_exti.h"
#include "stm32h5xx_ll_gpio.h"

#include "gpio.h"

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
void gpio_init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Outputs - Default value */
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

  /* Outputs - Push-Pull */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
}

/**
 * @brief This function handles EXTI Line0 interrupt.
 */
void EXTI0_IRQHandler(void)
{
  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_0) != RESET)
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_0);
  }
}

/* inputs */
bool button(void)
{
  return (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) == 0);
}

/* outputs */
void led_run(bool on)
{
  if (on)
  {
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
  }
  else
  {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
  }
}
