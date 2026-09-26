#include <stdbool.h>
#include <stdint.h>

#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_dma.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_usart.h"

#include "usart.h"

#define U5_BUF_TX_SIZE  1024
#define U5_BUF_RX_SIZE  1024

uint8_t u5_tx_buffer[U5_BUF_TX_SIZE];
uint8_t u5_rx_buffer[U5_BUF_RX_SIZE];

static volatile uint32_t u5_rx_wr_index = 0;
static volatile uint32_t u5_rx_rd_index = 0;
static volatile uint32_t u5_line_ready = 0;

static volatile uint32_t u5_tx_wr_index = 0;
static volatile uint32_t u5_tx_rd_index = 0;
static volatile uint32_t u5_tx_len = 0;

static void uart_init_common(USART_TypeDef *USARTx, uint32_t baud, bool swap)
{
    LL_USART_InitTypeDef USART_InitStruct = {0};

    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = baud;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USARTx, &USART_InitStruct);
    LL_USART_SetTXRXSwap(USARTx, swap ? LL_USART_TXRX_SWAPPED : LL_USART_TXRX_STANDARD);
    LL_USART_ConfigNodeAddress(USARTx, LL_USART_ADDRESS_DETECT_7B, 0x0a);
    LL_USART_SetTXFIFOThreshold(USARTx, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USARTx, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(USARTx);
    LL_USART_ConfigAsyncMode(USARTx);
    LL_USART_Enable(USARTx);
    LL_USART_EnableIT_RXNE(USARTx);
    LL_USART_EnableIT_CM(USARTx);
}

void u5_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_DMA_InitTypeDef DMA_InitStruct = {0};

    LL_RCC_SetUSARTClockSource(LL_RCC_UART5_CLKSOURCE_PCLK1);

    /* Peripheral clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPDMA1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_12 | LL_GPIO_PIN_13;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_14;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    DMA_InitStruct.SrcAddress = 0x00000000U;
    DMA_InitStruct.DestAddress = (uint32_t) & (UART5->TDR);
    DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    DMA_InitStruct.BlkHWRequest = LL_DMA_HWREQUEST_SINGLEBURST;
    DMA_InitStruct.DataAlignment = LL_DMA_DATA_ALIGN_ZEROPADD;
    DMA_InitStruct.SrcBurstLength = 1;
    DMA_InitStruct.DestBurstLength = 1;
    DMA_InitStruct.SrcDataWidth = LL_DMA_SRC_DATAWIDTH_BYTE;
    DMA_InitStruct.DestDataWidth = LL_DMA_DEST_DATAWIDTH_BYTE;
    DMA_InitStruct.SrcIncMode = LL_DMA_SRC_INCREMENT;
    DMA_InitStruct.DestIncMode = LL_DMA_DEST_FIXED;
    DMA_InitStruct.Priority = LL_DMA_LOW_PRIORITY_LOW_WEIGHT;
    DMA_InitStruct.BlkDataLength = 0x00000000U;
    DMA_InitStruct.TriggerMode = LL_DMA_TRIGM_BLK_TRANSFER;
    DMA_InitStruct.TriggerPolarity = LL_DMA_TRIG_POLARITY_MASKED;
    DMA_InitStruct.TriggerSelection = 0x00000000U;
    DMA_InitStruct.Request = LL_GPDMA1_REQUEST_UART5_TX;
    DMA_InitStruct.TransferEventMode = LL_DMA_TCEM_BLK_TRANSFER;
    DMA_InitStruct.Mode = LL_DMA_NORMAL;
    DMA_InitStruct.SrcAllocatedPort = LL_DMA_SRC_ALLOCATED_PORT0;
    DMA_InitStruct.DestAllocatedPort = LL_DMA_DEST_ALLOCATED_PORT0;
    DMA_InitStruct.LinkAllocatedPort = LL_DMA_LINK_ALLOCATED_PORT1;
    DMA_InitStruct.LinkStepMode = LL_DMA_LSM_FULL_EXECUTION;
    DMA_InitStruct.LinkedListBaseAddr = 0x00000000U;
    DMA_InitStruct.LinkedListAddrOffset = 0x00000000U;
    LL_DMA_Init(GPDMA1, LL_DMA_CHANNEL_0, &DMA_InitStruct);

    LL_DMA_EnableIT_TC(GPDMA1, LL_DMA_CHANNEL_0);
    LL_DMA_EnableIT_DTE(GPDMA1, LL_DMA_CHANNEL_0);

    NVIC_SetPriority(GPDMA1_Channel0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 10, 0));
    NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

    uart_init_common(UART5, 115200, false);

    NVIC_SetPriority(UART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 10, 0));
    NVIC_EnableIRQ(UART5_IRQn);
}

static void u5_tx(void)
{
    if (u5_tx_wr_index == u5_tx_rd_index)
    {
        return;
    }

    if (u5_tx_len != 0)
    {
        return;
    }

    u5_tx_len = (u5_tx_wr_index > u5_tx_rd_index) ? (u5_tx_wr_index - u5_tx_rd_index)
                                                   : (U5_BUF_TX_SIZE - u5_tx_rd_index);

    LL_DMA_SetSrcAddress(GPDMA1, LL_DMA_CHANNEL_0, (uint32_t) &u5_tx_buffer[u5_tx_rd_index]);
    LL_DMA_SetBlkDataLength(GPDMA1, LL_DMA_CHANNEL_0, u5_tx_len);

    LL_USART_EnableDMAReq_TX(UART5);
    LL_DMA_EnableChannel(GPDMA1, LL_DMA_CHANNEL_0);
}

void u5_write(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        u5_tx_buffer[u5_tx_wr_index] = data[i];
        if (++u5_tx_wr_index >= U5_BUF_TX_SIZE)
        {
            u5_tx_wr_index = 0;
        }
    }
    u5_tx();
}

void UART5_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_RXNE(UART5) || LL_USART_IsActiveFlag_ORE(UART5))
    {
        if (LL_USART_IsActiveFlag_ORE(UART5))
        {
            LL_USART_ClearFlag_ORE(UART5);
        }

        u5_rx_buffer[u5_rx_wr_index] = LL_USART_ReceiveData8(UART5);
        if (++u5_rx_wr_index >= U5_BUF_RX_SIZE)
        {
            u5_rx_wr_index = 0;
        }
    }
    if (LL_USART_IsActiveFlag_CM(UART5))
    {
        LL_USART_ClearFlag_CM(UART5);
        u5_line_ready = u5_rx_wr_index;
    }
}

void GPDMA1_Channel0_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC(GPDMA1, LL_DMA_CHANNEL_0))
    {
        LL_DMA_ClearFlag_TC(GPDMA1, LL_DMA_CHANNEL_0);
        u5_tx_rd_index += u5_tx_len;
        if (u5_tx_rd_index >= U5_BUF_TX_SIZE)
        {
            u5_tx_rd_index = 0;
        }
    }
    else if (LL_DMA_IsActiveFlag_DTE(GPDMA1, LL_DMA_CHANNEL_0))
    { // TODO log event?
        LL_DMA_ClearFlag_DTE(GPDMA1, LL_DMA_CHANNEL_0);
    }
    LL_DMA_DisableChannel(GPDMA1, LL_DMA_CHANNEL_0);

    u5_tx_len = 0;
    u5_tx();
}

uint32_t u5_line_status(void)
{
    return (u5_line_ready != 0xFFFF);
}

int32_t u5_read(void)
{
    int ret = -1;

    if (u5_rx_rd_index != u5_line_ready)
    {
        ret = u5_rx_buffer[u5_rx_rd_index];
        if (++u5_rx_rd_index >= U5_BUF_RX_SIZE)
        {
            u5_rx_rd_index = 0;
        }
    }
    else
    {
        u5_line_ready = 0xFFFF;
    }

    return ret;
}
