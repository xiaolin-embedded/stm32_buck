/*
 * app_freertos.c
 *
 * FreeRTOS object creation and kernel hooks.  The buck control loop does not
 * run as a task: it lives in the HRTIM interrupt at 100 kHz.  Tasks only do
 * slow housekeeping (serial shell).
 */

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include "app_power.h"
#include "app_shell.h"

static osThreadId_t s_shell_task;

static const osThreadAttr_t s_shell_attr = {
    .name       = "shell",
    .stack_size = 2048u,
    .priority   = osPriorityNormal,
};

void MX_FREERTOS_Init(void)
{
    s_shell_task = osThreadNew(Shell_Task, NULL, &s_shell_attr);

    if (s_shell_task == NULL)
    {
        Error_Handler();
    }
}

/* -------------------------------------------------------------------------- */
/* Hooks                                                                      */
/* -------------------------------------------------------------------------- */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    /* Stop the converter before spinning. */
    Power_EmergencyStop();
    __disable_irq();
    for (;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    Power_EmergencyStop();
    __disable_irq();
    for (;;)
    {
    }
}
