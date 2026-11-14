#ifndef APP_POWER_H
#define APP_POWER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    PWR_STATE_OFF = 0,
    PWR_STATE_SOFT_START,
    PWR_STATE_RUN,
    PWR_STATE_FAULT
} PowerState_t;

#define PWR_FAULT_NONE      0u
#define PWR_FAULT_VIN_UV    (1u << 0)
#define PWR_FAULT_VIN_OV    (1u << 1)
#define PWR_FAULT_VOUT_OV   (1u << 2)
#define PWR_FAULT_IOUT_OC   (1u << 3)
#define PWR_FAULT_PEAK_OC   (1u << 4)

typedef struct
{
    uint32_t state;         /* PowerState_t */
    uint32_t fault;         /* PWR_FAULT_* bit mask */
    uint32_t vin_mv;
    uint32_t vout_mv;
    uint32_t iout_ma;
    uint32_t pout_mw;
    uint32_t vset_mv;
    uint32_t ilim_ma;
    uint16_t duty_permille; /* 0..1000 */
    uint32_t cc_active;     /* 1 = current loop is limiting (CC) */
    uint32_t loop_count;
} PowerTelemetry_t;

/* Initialise HRTIM, ADC and the control interrupt. Output stays disabled. */
void Power_Init(void);

/* Request output on/off. Start is refused while a fault is latched. */
bool Power_Start(void);
void Power_Stop(void);

/* Clear latched faults. Only allowed while the output is stopped. */
bool Power_ClearFault(void);

/* Setpoints. Values are validated against the VSET / ILIM limits. */
bool Power_SetVout_mV(uint32_t mv);
bool Power_SetIlimit_mA(uint32_t ma);
uint32_t Power_GetVoutSet_mV(void);

float Power_GetVinGain(void);
float Power_GetVoutGain(void);
float Power_GetIoutGain(void);
uint32_t Power_GetIlimit_mA(void);

/* Atomic snapshot of the latest control-loop data. */
void Power_GetTelemetry(PowerTelemetry_t *out);

const char *Power_StateName(uint32_t state);
const char *Power_FaultName(uint32_t fault);

/* Called from HRTIM1_TIMA_IRQHandler(). */
void Power_IRQHandler(void);

/* Immediately force both gate outputs low. Safe to call before Power_Init(). */
void Power_EmergencyStop(void);

#endif /* APP_POWER_H */
