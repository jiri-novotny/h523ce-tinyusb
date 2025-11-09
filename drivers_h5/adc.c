#include <string.h>

#include "stm32h5xx_ll_adc.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_rcc.h"

#include "adc.h"

extern void Error_Handler(void);

#define ADC_CALIBRATION_TIMEOUT_MS (1UL)
#define ADC_ENABLE_TIMEOUT_MS      (1UL)

/* Delay between ADC end of calibration and ADC enable.                     */
/* Delay estimation in CPU cycles: Case of ADC enable done                  */
/* immediately after ADC calibration, ADC clock setting slow                */
/* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
/* (CPU clock / ADC clock) is above 32.                                     */
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)

#define AD_CHANNELS                       2

static uint16_t adc_data[AD_CHANNELS];
static uint8_t adc_ptr;
static uint8_t adc_done;

static void adc_activate(void)
{
  __IO uint32_t wait_loop_index = 0U;
  uint32_t timeout = 0U; /* Variable used for timeout management */

  /*## Operation on ADC hierarchical scope: ADC instance #####################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 series, setting of these features is conditioned to  */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Run ADC self calibration */
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);

    /* Poll for ADC effectively calibrated */
    timeout = ADC_CALIBRATION_TIMEOUT_MS;

    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if (timeout-- == 0)
        {
          /* Error: Time-out */
          Error_Handler();
        }
      }
    }

    /* Delay between ADC end of calibration and ADC enable.                   */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while (wait_loop_index != 0)
    {
      wait_loop_index--;
    }

    /* Enable ADC */
    LL_ADC_Enable(ADC1);

    /* Poll for ADC ready to convert */
    timeout = ADC_ENABLE_TIMEOUT_MS;

    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
    {
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if (timeout-- == 0)
        {
          /* Error: Time-out */
          Error_Handler();
        }
      }
    }

    /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
    /*       status afterwards.                                               */
    /*       This flag should be cleared at ADC Deactivation, before a new    */
    /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
  }
}

/* ADC1 init function */
void adc_init(void)
{
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_RCC_SetADCDACClockSource(LL_RCC_ADCDAC_CLKSOURCE_HSE);

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);

  /* ADC1 interrupt Init */
  NVIC_SetPriority(ADC1_IRQn, 8);
  NVIC_EnableIRQ(ADC1_IRQn);

  /* Configure the ADC multi-mode */
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  /* Common config */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetSamplingMode(ADC1, LL_ADC_REG_SAMPLING_MODE_NORMAL);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while (wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /* Configure Regular Channel */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_92CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_TEMPSENSOR);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);

  /* Configuration of ADC interruptions */
  /* Enable interruption ADC group regular end of unitary conversion */
  LL_ADC_EnableIT_EOC(ADC1);
  LL_ADC_EnableIT_EOS(ADC1);

  /* Configuration of ADC interruptions */
  /* Enable interruption ADC group regular overrun */
  LL_ADC_EnableIT_OVR(ADC1);

  adc_activate();
  adc_done = 0;
}

void adc_start(void)
{
  if ((LL_ADC_IsEnabled(ADC1) == 1) && (LL_ADC_IsDisableOngoing(ADC1) == 0) &&
      (LL_ADC_REG_IsConversionOngoing(ADC1) == 0) && adc_done == 0)
  {
    adc_ptr = 0;
    LL_ADC_REG_StartConversion(ADC1);
  }
}

void ADC1_IRQHandler(void)
{
  /* Check whether ADC group regular end of unitary conversion caused         */
  /* the ADC interruption.                                                    */
  if (LL_ADC_IsActiveFlag_EOC(ADC1) != 0)
  {
    /* Clear flag ADC group regular end of unitary conversion */
    LL_ADC_ClearFlag_EOC(ADC1);

    /* Retrieve ADC conversion data */
    adc_data[adc_ptr++] = LL_ADC_REG_ReadConversionData12(ADC1);
  }

  if (LL_ADC_IsActiveFlag_EOS(ADC1) != 0)
  {
    LL_ADC_ClearFlag_EOS(ADC1);

    adc_done = 1;
  }

  /* Check whether ADC group regular overrun caused the ADC interruption */
  if (LL_ADC_IsActiveFlag_OVR(ADC1) != 0)
  {
    /* Clear flag ADC group regular overrun */
    LL_ADC_ClearFlag_OVR(ADC1);

    /* In case of error due to overrun: Disable ADC group regular overrun interruption */
    LL_ADC_DisableIT_OVR(ADC1);

    /* Error reporting */
    Error_Handler();
  }
}

bool adc_complete(void)
{
  bool ret = (adc_done == 1);
  adc_done = 0;
  return ret;
}

void adc_send(void)
{
#if 0
  uint8_t pkt[8];
  uint16_t ref;
  uint16_t temp;

  pkt[HDR] = HDR_TEMP;
  pkt[LEN] = 6;
  ref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(adc_data[0], LL_ADC_RESOLUTION_12B);
  temp = __LL_ADC_CALC_TEMPERATURE(TEMPSENSOR_CAL_VREFANALOG, adc_data[1], LL_ADC_RESOLUTION_12B);
  memcpy(&pkt[DATA], &temp, 2);
  memcpy(&pkt[DATA + 2], &ref, 2);
  memcpy(&pkt[DATA + 4], &adc_data[1], 2);

  usb_send(pkt, 8);
#endif
}
