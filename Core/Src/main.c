/*
 * main.c
 *
 * DPOS G4 BUCK V2 digital power supply.
 *
 *   - 170 MHz from the 8 MHz board crystal (HSI fallback)
 *   - 200 kHz HRTIM buck with 100 kHz CC/CV control loop
 *   - FreeRTOS: UART shell + 1 Hz telemetry monitor
 *
 * The output is disabled after reset; use the shell ('on') to start it.
 */

#include "main.h"
#include "cmsis_os.h"

#include "app_config.h"
#include "app_power.h"
#include "app_shell.h"

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    Power_Init();       /* HRTIM + ADC, outputs off */
    Shell_Init();       /* USART2, RX interrupt */

    if (osKernelInitialize() != osOK)
    {
        Error_Handler();
    }

    MX_FREERTOS_Init();

    if (osKernelStart() != osOK)
    {
        Error_Handler();
    }

    Error_Handler();
    return 0;
}

/*
 * 170 MHz from the 8 MHz board crystal: 8 / 2 * 85 / 2 = 170 MHz.
 * The schematic says 12 MHz, but the assembled board uses 8 MHz.  If the
 * crystal fails to start, fall back to HSI: 16 / 4 * 85 / 2 = 170 MHz.
 * APB2 is divided by 2 so the ADC clock stays at 21.25 MHz.
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = RCC_PLLM_DIV2;
    osc.PLL.PLLN       = 85;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = RCC_PLLQ_DIV2;
    osc.PLL.PLLR       = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        /* Crystal missing or not starting: use the internal oscillator. */
        osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState            = RCC_HSI_ON;
        osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM            = RCC_PLLM_DIV4;
        if (HAL_RCC_OscConfig(&osc) != HAL_OK)
        {
            Error_Handler();
        }
    }

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;   /* 170 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;   /*  85 MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* RGB LED off (active low). */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_SET);

    gpio.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void Error_Handler(void)
{
    Power_EmergencyStop();
    __disable_irq();
    for (;;)
    {
    }
}
