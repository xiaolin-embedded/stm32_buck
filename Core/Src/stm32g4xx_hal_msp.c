/*
 * stm32g4xx_hal_msp.c
 *
 * MCU support package for the peripherals used by this project.  All
 * peripheral clock, GPIO and DMA setup lives here; the application modules
 * only deal with the HAL handles.
 */

#include "main.h"

static DMA_HandleTypeDef hdma_adc2;

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* UCPD dead-battery pull-ups are not needed on this board. */
    HAL_PWREx_DisableUCPDDeadBattery();
}

/* -------------------------------------------------------------------------- */
/* HRTIM                                                                      */
/* -------------------------------------------------------------------------- */
void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef *hhrtim)
{
    if (hhrtim->Instance == HRTIM1)
    {
        __HAL_RCC_HRTIM1_CLK_ENABLE();
    }
}

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim)
{
    GPIO_InitTypeDef gpio = {0};

    if (hhrtim->Instance != HRTIM1)
    {
        return;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;   /* HRTIM_CHA1 / CHA2 */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLDOWN;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/* -------------------------------------------------------------------------- */
/* ADC2 + DMA1 channel 3                                                      */
/* -------------------------------------------------------------------------- */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef gpio = {0};

    if (hadc->Instance != ADC2)
    {
        return;
    }

    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA1 = Iout, PA6 = Vout, PA7 = Vin */
    gpio.Pin  = GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    hdma_adc2.Instance                 = DMA1_Channel3;
    hdma_adc2.Init.Request             = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode                = DMA_CIRCULAR;
    hdma_adc2.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc2);
}

/* -------------------------------------------------------------------------- */
/* DAC1 + COMP3 peak-current comparator                                       */
/* -------------------------------------------------------------------------- */
void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
    GPIO_InitTypeDef gpio = {0};

    if (hdac->Instance != DAC1)
    {
        return;
    }

    __HAL_RCC_DAC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin  = GPIO_PIN_4;                 /* DAC1_CH1 / COMP3_INM path */
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void HAL_COMP_MspInit(COMP_HandleTypeDef *hcomp)
{
    GPIO_InitTypeDef gpio = {0};

    if (hcomp->Instance != COMP3)
    {
        return;
    }

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin  = GPIO_PIN_0;                 /* COMP3_INP = I_PEAK_DETC */
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/* -------------------------------------------------------------------------- */
/* USART2 shell                                                               */
/* -------------------------------------------------------------------------- */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    if (huart->Instance != USART2)
    {
        return;
    }

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_2;                /* TX */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin  = GPIO_PIN_3;                     /* RX */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}
