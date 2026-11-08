/*
 * app_power.c
 *
 * 200 kHz HRTIM buck converter with a 100 kHz average-current / voltage
 * double loop.  The control law is the incremental PI structure used by the
 * vendor reference examples (P3/P7), running entirely inside the HRTIM
 * repetition interrupt.  FreeRTOS tasks only consume telemetry and change
 * setpoints.
 */

#include "main.h"
#include "app_config.h"
#include "app_power.h"

/* -------------------------------------------------------------------------- */
/* Handles                                                                    */
/* -------------------------------------------------------------------------- */
HRTIM_HandleTypeDef hhrtim1;
ADC_HandleTypeDef  hadc2;

/* -------------------------------------------------------------------------- */
/* ADC results: rank1 = Vout (PA6), rank2 = Iout (PA1), rank3 = Vin (PA7)      */
/* -------------------------------------------------------------------------- */
static volatile uint16_t s_adc_dma[3];

/* -------------------------------------------------------------------------- */
/* Setpoints (written by tasks, read by the control ISR)                      */
/* -------------------------------------------------------------------------- */
static volatile uint32_t s_vset_mv   = DEFAULT_VSET_MV;
static volatile uint32_t s_ilim_ma   = DEFAULT_ILIM_MA;
static volatile uint32_t s_start_req = 0u;
static volatile uint32_t s_fault     = PWR_FAULT_NONE;
static volatile uint32_t s_state     = PWR_STATE_OFF;

/* -------------------------------------------------------------------------- */
/* Control state                                                              */
/* -------------------------------------------------------------------------- */
static float s_vref_mv;
static float s_du_v;
static float s_du_i;
static float s_verr;
static float s_verr_prev;
static float s_dverr;
static float s_dverr_prev;
static float s_ierr;
static float s_ierr_prev;
static float s_dierr;
static float s_dierr_prev;
static float s_duty;

/* First-order IIR (3/4 old + 1/4 new) on each ADC channel. */
static uint32_t s_filt_v;
static uint32_t s_filt_i;
static uint32_t s_filt_in;
static bool     s_filt_valid;
static volatile float s_vin_gain  = CAL_VIN_GAIN;
static volatile float s_vout_gain = CAL_VOUT_GAIN;
static volatile float s_iout_gain = CAL_IOUT_GAIN;

/* Telemetry snapshot guarded by a sequence counter */
static volatile uint32_t s_seq;
static volatile PowerTelemetry_t s_telem;
static uint32_t s_loop_count;
static uint32_t s_led_ms;
static uint32_t s_led_div;
static bool     s_cc_active;
static uint32_t s_oc_count;
static volatile uint32_t s_hrtim_ready;

/* -------------------------------------------------------------------------- */
/* Small helpers                                                              */
/* -------------------------------------------------------------------------- */
static inline void outputs_on(void)
{
    HRTIM1->sCommonRegs.OENR = HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2;
}

static inline void outputs_off(void)
{
    HRTIM1->sCommonRegs.ODISR = HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2;
}

static inline void set_duty(float duty)
{
    uint32_t cmp;

    if (duty < DUTY_MIN) duty = DUTY_MIN;
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    s_duty = duty;

    cmp = (uint32_t)(duty * (float)PWM_PERIOD_COUNTS + 0.5f);
    if (cmp < 1u) cmp = 1u;
    if (cmp >= PWM_PERIOD_COUNTS) cmp = PWM_PERIOD_COUNTS - 1u;

    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = cmp;
    /* Sample in the middle of the on-time. */
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR = cmp >> 1;
}

static void filter_reset(void)
{
    s_filt_v = 0u;
    s_filt_i = 0u;
    s_filt_in = 0u;
    s_filt_valid = false;
}

static void reset_control(void)
{
    s_vref_mv     = 0.0f;
    s_du_v        = 0.0f;
    s_du_i        = 0.0f;
    s_verr        = 0.0f;
    s_verr_prev   = 0.0f;
    s_dverr       = 0.0f;
    s_dverr_prev  = 0.0f;
    s_ierr        = 0.0f;
    s_ierr_prev   = 0.0f;
    s_dierr       = 0.0f;
    s_dierr_prev  = 0.0f;
    s_duty        = 0.0f;
    s_oc_count    = 0u;
}

static void led_write(bool red, bool green, bool blue)
{
    /* Active low: PB0 red, PB1 green, PB2 blue. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, red   ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, green ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, blue  ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* Called every 10 ms from the control ISR.
 *
 *   blue : 2 Hz heartbeat in every state (firmware alive)
 *   green: steady while the output is enabled (SOFT_START or RUN)
 *   red  : steady on a latched fault
 */
static void led_update(void)
{
    bool red = false;
    bool green = false;
    bool blue;
    uint32_t t = s_led_ms;

    blue = ((t % 500u) < 100u);

    if (s_state == PWR_STATE_FAULT)
    {
        red = true;
    }
    else if ((s_state == PWR_STATE_SOFT_START) || (s_state == PWR_STATE_RUN))
    {
        green = true;
    }

    led_write(red, green, blue);
}

/* -------------------------------------------------------------------------- */
/* Scaling                                                                    */
/* -------------------------------------------------------------------------- */
static inline uint32_t adc_to_vin_mv(uint16_t raw)
{
    /* raw * 3300 / 4096 * 17.5 = raw * 577500 / 40960 (fits in 32 bits) */
    return ((uint32_t)raw * 577500u) / 40960u;
}

static inline uint32_t adc_to_vout_mv(uint16_t raw)
{
    int32_t mv = (int32_t)adc_to_vin_mv(raw) - CAL_VOUT_OFFSET_MV;
    return (mv > 0) ? (uint32_t)mv : 0u;
}

static inline uint32_t adc_to_iout_ma(uint16_t raw)
{
    /* ADC mV / (525 mV/A) -> mA */
    uint32_t mv = ((uint32_t)raw * 3300u) / 4096u;
    int32_t  ma = (int32_t)((mv * 1000u + 262u) / 525u) - CAL_IOUT_OFFSET_MA;
    return (ma > 0) ? (uint32_t)ma : 0u;
}

/* -------------------------------------------------------------------------- */
/* Control step, called at 100 kHz                                            */
/* -------------------------------------------------------------------------- */
static void control_step(uint32_t vin_mv, uint32_t vout_mv, uint32_t iout_ma)
{
    uint32_t vset = s_vset_mv;
    uint32_t ilim = s_ilim_ma;
    uint32_t vout_limit;
    uint32_t iout_limit;

    /* Latched fault or output disabled -> safe state. */
    if (s_start_req == 0u)
    {
        s_state = PWR_STATE_OFF;
        outputs_off();
        reset_control();
        s_cc_active = false;
        return;
    }

    if (s_fault != PWR_FAULT_NONE)
    {
        s_state = PWR_STATE_FAULT;
        outputs_off();
        reset_control();
        s_cc_active = false;
        return;
    }

    /* Input window and output protections. */
    /* During a setpoint decrease the output is still at the old voltage.
     * Follow the active reference (or the target if it is higher) so a
     * legitimate downward change does not look like an over-voltage. */
    {
        float limit_ref = (s_vref_mv > (float)vset) ? s_vref_mv : (float)vset;
        vout_limit = (uint32_t)limit_ref + VOUT_OV_MARGIN_MV;
        if (vout_limit > VOUT_OV_ABS_MV) vout_limit = VOUT_OV_ABS_MV;
    }
    iout_limit = ilim + IOC_MARGIN_MA;
    if (iout_limit > IOC_ABS_MA) iout_limit = IOC_ABS_MA;

    if (vin_mv < VIN_UV_MV)
    {
        s_fault = PWR_FAULT_VIN_UV;
    }
    else if (vin_mv > VIN_OV_MV)
    {
        s_fault = PWR_FAULT_VIN_OV;
    }
    else if (vout_mv > vout_limit)
    {
        s_fault = PWR_FAULT_VOUT_OV;
    }
    else if (iout_ma > IOC_FAST_MA)
    {
        /* Hard short / current-loop failure: trip immediately. */
        s_fault = PWR_FAULT_IOUT_OC;
    }
    else if (iout_ma > iout_limit)
    {
        /* A load step into CC briefly exceeds ilim+margin while the current
         * loop takes over.  Latch only if it persists. */
        s_oc_count++;
        if (s_oc_count >= IOC_DEBOUNCE_TICKS)
        {
            s_fault = PWR_FAULT_IOUT_OC;
        }
    }
    else
    {
        s_oc_count = 0u;
    }

    if (s_fault != PWR_FAULT_NONE)
    {
        s_state = PWR_STATE_FAULT;
        outputs_off();
        reset_control();
        s_cc_active = false;
        return;
    }

    /* Start or keep running. */
    if (s_state == PWR_STATE_OFF || s_state == PWR_STATE_FAULT)
    {
        reset_control();
        set_duty(DUTY_MIN);
        outputs_on();
        s_state = PWR_STATE_SOFT_START;
    }

    /* Reference ramp: up at VREF_RAMP_MV_TICK, down immediately. */
    if (s_vref_mv < (float)vset)
    {
        s_vref_mv += VREF_RAMP_MV_TICK;
        if (s_vref_mv > (float)vset) s_vref_mv = (float)vset;
    }
    else if (s_vref_mv > (float)vset)
    {
        /* Ramp down at the same rate as soft start instead of stepping. */
        s_vref_mv -= VREF_RAMP_MV_TICK;
        if (s_vref_mv < (float)vset) s_vref_mv = (float)vset;
    }
    else
    {
        /* equal */
    }

    if (s_state == PWR_STATE_SOFT_START && s_vref_mv >= (float)vset)
    {
        s_state = PWR_STATE_RUN;
    }

    /* Outer voltage loop (incremental PI(D)). */
    s_verr  = (s_vref_mv - (float)vout_mv) * 0.001f;
    s_dverr = s_verr - s_verr_prev;
    s_du_v += KP_V * s_dverr + KI_V * s_verr + KD_V * (s_dverr - s_dverr_prev);
    s_verr_prev   = s_verr;
    s_dverr_prev  = s_dverr;

    if (s_du_v > OSC_LIMIT) s_du_v = OSC_LIMIT;
    else if (s_du_v < 0.0f) s_du_v = 0.0f;

    /* Inner average-current loop. */
    s_ierr  = ((float)ilim - (float)iout_ma) * 0.001f;
    s_dierr = s_ierr - s_ierr_prev;
    s_du_i += KP_I * s_dierr + KI_I * s_ierr + KD_I * (s_dierr - s_dierr_prev);
    s_ierr_prev  = s_ierr;
    s_dierr_prev = s_dierr;

    if (s_du_i > OSC_LIMIT) s_du_i = OSC_LIMIT;
    else if (s_du_i < 0.0f) s_du_i = 0.0f;

    /* Whichever loop asks for less duty wins (CV or CC). */
    {
        float duty_v = s_du_v / VOSC;
        float duty_i = s_du_i / VOSC;
        set_duty((duty_v < duty_i) ? duty_v : duty_i);
    }

    /* CC/CV indication with a small hysteresis. */
    if (s_state == PWR_STATE_RUN)
    {
        if (s_du_i + 0.02f < s_du_v)
        {
            s_cc_active = true;
        }
        else if (s_du_i > s_du_v + 0.05f)
        {
            s_cc_active = false;
        }
    }
    else
    {
        s_cc_active = false;
    }
}

/* -------------------------------------------------------------------------- */
/* Interrupt handler                                                          */
/* -------------------------------------------------------------------------- */
void Power_IRQHandler(void)
{
    uint32_t vin_mv;
    uint32_t vout_mv;
    uint32_t iout_ma;
    uint32_t pout_mw;

    if ((HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].TIMxDIER & HRTIM_TIM_IT_REP) == 0u)
    {
        return;
    }
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].TIMxICR = HRTIM_TIM_IT_REP;

    /* Read each DMA value exactly once, then filter. */
    {
        uint16_t raw_v  = s_adc_dma[0];
        uint16_t raw_i  = s_adc_dma[1];
        uint16_t raw_in = s_adc_dma[2];

        if (!s_filt_valid)
        {
            s_filt_v  = raw_v;
            s_filt_i  = raw_i;
            s_filt_in = raw_in;
            s_filt_valid = true;
        }
        else
        {
            s_filt_v  = (s_filt_v  * 3u + raw_v  + 2u) >> 2;
            s_filt_i  = (s_filt_i  * 3u + raw_i  + 2u) >> 2;
            s_filt_in = (s_filt_in * 3u + raw_in + 2u) >> 2;
        }

        vout_mv = (uint32_t)((float)adc_to_vout_mv((uint16_t)s_filt_v) * s_vout_gain + 0.5f);
        iout_ma = (uint32_t)((float)adc_to_iout_ma((uint16_t)s_filt_i) * s_iout_gain + 0.5f);
        if (iout_ma < IOUT_ZERO_DEADBAND_MA)
        {
            iout_ma = 0u;       /* kill amplifier offset / ADC quantization */
        }
        vin_mv  = (uint32_t)((float)adc_to_vin_mv((uint16_t)s_filt_in) * s_vin_gain + 0.5f);
    }

    /* Guard the power product against an out-of-range conversion. */
    if ((vout_mv <= 60000u) && (iout_ma <= 10000u))
    {
        pout_mw = (vout_mv * iout_ma) / 1000u;
    }
    else
    {
        pout_mw = 0u;
    }

    control_step(vin_mv, vout_mv, iout_ma);

    /* Publish a consistent snapshot for the tasks. */
    s_seq++;
    __DMB();
    s_telem.state         = s_state;
    s_telem.fault         = s_fault;
    s_telem.vin_mv        = vin_mv;
    s_telem.vout_mv       = vout_mv;
    s_telem.iout_ma       = iout_ma;
    s_telem.pout_mw       = pout_mw;
    s_telem.vset_mv       = s_vset_mv;
    s_telem.ilim_ma       = s_ilim_ma;
    s_telem.duty_permille = (uint16_t)(s_duty * 1000.0f);
    s_telem.cc_active     = s_cc_active ? 1u : 0u;
    s_telem.loop_count    = ++s_loop_count;
    __DMB();
    s_seq++;

    /* 100 kHz / 1000 = 10 ms LED time base. */
    if (++s_led_div >= 1000u)
    {
        s_led_div = 0u;
        s_led_ms += 10u;
        led_update();
    }
}

void HRTIM1_TIMA_IRQHandler(void)
{
    Power_IRQHandler();
}

/* -------------------------------------------------------------------------- */
/* Initialisation                                                             */
/* -------------------------------------------------------------------------- */
static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc2.Instance                      = ADC2;
    hadc2.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;  /* 85 MHz / 4 */
    hadc2.Init.Resolution               = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
    hadc2.Init.GainCompensation         = 0;
    hadc2.Init.ScanConvMode             = ADC_SCAN_ENABLE;
    hadc2.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait         = DISABLE;
    hadc2.Init.ContinuousConvMode       = DISABLE;                   /* one scan per HRTIM trigger */
    hadc2.Init.NbrOfConversion          = 3;
    hadc2.Init.DiscontinuousConvMode    = DISABLE;
    hadc2.Init.NbrOfDiscConversion      = 0;
    hadc2.Init.ExternalTrigConv         = ADC_EXTERNALTRIG_HRTIM_TRG1;
    hadc2.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2.Init.DMAContinuousRequests    = ENABLE;
    hadc2.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    hadc2.Init.OversamplingMode         = DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }

    /* Rank 1: Vout (PA6 / ADC2_IN3) */
    sConfig.Channel      = ADC_CHANNEL_3;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Rank 2: Iout (PA1 / ADC2_IN2) */
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank    = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Rank 3: Vin (PA7 / ADC2_IN4) */
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank    = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc_dma, 3) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_HRTIM1_Init(void)
{
    HRTIM_TimeBaseCfgTypeDef   pTimeBaseCfg  = {0};
    HRTIM_TimerCfgTypeDef      pTimerCfg     = {0};
    HRTIM_TimerCtlTypeDef      pTimerCtl     = {0};
    HRTIM_OutputCfgTypeDef     pOutputCfg    = {0};
    HRTIM_CompareCfgTypeDef    pCompareCfg   = {0};
    HRTIM_DeadTimeCfgTypeDef   pDeadTimeCfg  = {0};
    HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};

    hhrtim1.Instance = HRTIM1;
    hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
    hhrtim1.Init.SyncOptions             = HRTIM_SYNCOPTION_NONE;
    if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
    {
        Error_Handler();
    }
    s_hrtim_ready = 1u;   /* clock is running; emergency stop is now safe */

    /* High-resolution mode needs a calibrated DLL (present in the CubeMX
     * generated init and required before the x32 prescaler is used). */
    if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
    {
        Error_Handler();
    }

    /* 200 kHz: 5.44 GHz equivalent / 27200 */
    pTimeBaseCfg.Period            = PWM_PERIOD_COUNTS;
    pTimeBaseCfg.RepetitionCounter = 0x01;                  /* control IRQ every 2 PWM periods */
    pTimeBaseCfg.PrescalerRatio    = HRTIM_PRESCALERRATIO_MUL32;
    pTimeBaseCfg.Mode              = HRTIM_MODE_CONTINUOUS;
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
    {
        Error_Handler();
    }

    /* Master timer only drives the period event. */
    pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
    pTimerCfg.DMARequests       = HRTIM_MASTER_DMA_NONE;
    pTimerCfg.DMASrcAddress     = 0x0000;
    pTimerCfg.DMADstAddress     = 0x0000;
    pTimerCfg.DMASize           = 0x1;
    pTimerCfg.HalfModeEnable    = HRTIM_HALFMODE_DISABLED;
    pTimerCfg.InterleavedMode   = HRTIM_INTERLEAVED_MODE_DISABLED;
    pTimerCfg.StartOnSync       = HRTIM_SYNCSTART_ENABLED;
    pTimerCfg.ResetOnSync       = HRTIM_SYNCRESET_ENABLED;
    pTimerCfg.DACSynchro        = HRTIM_DACSYNC_NONE;
    pTimerCfg.PreloadEnable     = HRTIM_PRELOAD_ENABLED;
    pTimerCfg.UpdateGating      = HRTIM_UPDATEGATING_INDEPENDENT;
    pTimerCfg.BurstMode         = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
    pTimerCfg.RepetitionUpdate  = HRTIM_UPDATEONREPETITION_DISABLED;
    pTimerCfg.PushPull          = HRTIM_TIMPUSHPULLMODE_DISABLED;
    pTimerCfg.FaultEnable       = HRTIM_TIMFAULTENABLE_NONE;
    pTimerCfg.FaultLock         = HRTIM_TIMFAULTLOCK_READWRITE;
    pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
    pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
    pTimerCfg.UpdateTrigger     = HRTIM_TIMUPDATETRIGGER_MASTER;
    pTimerCfg.ResetTrigger      = HRTIM_TIMRESETTRIGGER_MASTER_PER;
    pTimerCfg.ResetUpdate       = HRTIM_TIMUPDATEONRESET_ENABLED;
    pTimerCfg.ReSyncUpdate      = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
    if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
    {
        Error_Handler();
    }

    /* Timer A: complementary outputs with dead time. */
    pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
    pTimerCfg.DMARequests       = HRTIM_TIM_DMA_NONE;
    pTimerCfg.DACSynchro        = HRTIM_DACSYNC_NONE;
    pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
    pTimerCfg.UpdateTrigger     = HRTIM_TIMUPDATETRIGGER_MASTER;
    pTimerCfg.ResetTrigger      = HRTIM_TIMRESETTRIGGER_MASTER_PER;
    pTimerCfg.ResetUpdate       = HRTIM_TIMUPDATEONRESET_ENABLED;
    if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
    {
        Error_Handler();
    }

    pTimerCtl.UpDownMode        = HRTIM_TIMERUPDOWNMODE_UP;
    pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
    if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
    {
        Error_Handler();
    }

    pOutputCfg.Polarity              = HRTIM_OUTPUTPOLARITY_HIGH;
    pOutputCfg.SetSource             = HRTIM_OUTPUTSET_TIMPER;
    pOutputCfg.ResetSource           = HRTIM_OUTPUTRESET_TIMCMP1;
    pOutputCfg.IdleMode              = HRTIM_OUTPUTIDLEMODE_NONE;
    pOutputCfg.IdleLevel             = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    pOutputCfg.FaultLevel            = HRTIM_OUTPUTFAULTLEVEL_NONE;
    pOutputCfg.ChopperModeEnable     = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
    {
        Error_Handler();
    }

    /* This HAL does not call the post-init hook from WaveformOutputConfig;
     * PA8/PA9 must be switched to AF13 here or the gate driver sees nothing. */
    HAL_HRTIM_MspPostInit(&hhrtim1);

    pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    pCompareCfg.CompareValue    = 100u;
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
    {
        Error_Handler();
    }

    /* ~118 ns dead time at fHRTIM = 170 MHz. */
    pDeadTimeCfg.Prescaler       = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1;
    pDeadTimeCfg.RisingValue     = 20u;
    pDeadTimeCfg.RisingSign      = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
    pDeadTimeCfg.RisingSignLock  = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_READONLY;
    pDeadTimeCfg.RisingLock      = HRTIM_TIMDEADTIME_RISINGLOCK_READONLY;
    pDeadTimeCfg.FallingValue    = 20u;
    pDeadTimeCfg.FallingSign     = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
    pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_READONLY;
    pDeadTimeCfg.FallingLock     = HRTIM_TIMDEADTIME_FALLINGLOCK_READONLY;
    if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg) != HAL_OK)
    {
        Error_Handler();
    }

    /* ADC trigger 1 on timer A compare 3 (mid on-time). */
    pADCTriggerCfg.Trigger      = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP3;
    pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
    if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_MASTER) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A) != HAL_OK)
    {
        Error_Handler();
    }

    /* Let the counters/DLL settle before the output crossbar is used. */
    HAL_Delay(1);

    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = 100u;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR = 50u;

    /* Outputs stay disabled until Power_Start(). */
    outputs_off();

    __HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_TIM_IT_REP);
    HAL_NVIC_SetPriority(HRTIM1_TIMA_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);
}

void Power_Init(void)
{
    reset_control();
    filter_reset();
    s_state = PWR_STATE_OFF;
    s_fault = PWR_FAULT_NONE;
    s_start_req = 0u;
    s_loop_count = 0u;
    s_led_ms = 0u;
    s_led_div = 0u;
    s_cc_active = false;
    led_update();

    MX_ADC2_Init();
    MX_HRTIM1_Init();
}

/* -------------------------------------------------------------------------- */
/* Public control API                                                         */
/* -------------------------------------------------------------------------- */
bool Power_Start(void)
{
    if (s_fault != PWR_FAULT_NONE)
    {
        return false;
    }
    s_start_req = 1u;
    return true;
}

void Power_Stop(void)
{
    s_start_req = 0u;
}

bool Power_ClearFault(void)
{
    if (s_start_req != 0u)
    {
        return false;
    }
    s_fault = PWR_FAULT_NONE;
    return true;
}

bool Power_SetVout_mV(uint32_t mv)
{
    if (mv < VSET_MIN_MV || mv > VSET_MAX_MV)
    {
        return false;
    }
    s_vset_mv = mv;
    return true;
}

bool Power_SetIlimit_mA(uint32_t ma)
{
    if (ma > ILIM_MAX_MA)
    {
        return false;
    }
    s_ilim_ma = ma;
    return true;
}

uint32_t Power_GetVoutSet_mV(void)
{
    return s_vset_mv;
}

uint32_t Power_GetIlimit_mA(void)
{
    return s_ilim_ma;
}

void Power_GetTelemetry(PowerTelemetry_t *out)
{
    uint32_t s1;
    uint32_t s2;

    if (out == NULL)
    {
        return;
    }

    do
    {
        s1 = s_seq;
        __DMB();
        *out = s_telem;
        __DMB();
        s2 = s_seq;
    } while (((s1 & 1u) != 0u) || (s1 != s2));
}

const char *Power_StateName(uint32_t state)
{
    switch (state)
    {
        case PWR_STATE_OFF:        return "OFF";
        case PWR_STATE_SOFT_START: return "SOFT";
        case PWR_STATE_RUN:        return "RUN";
        case PWR_STATE_FAULT:      return "FAULT";
        default:                   return "?";
    }
}

const char *Power_FaultName(uint32_t fault)
{
    if (fault == PWR_FAULT_NONE)  return "none";
    if (fault == PWR_FAULT_VIN_UV) return "VIN_UV";
    if (fault == PWR_FAULT_VIN_OV) return "VIN_OV";
    if (fault == PWR_FAULT_VOUT_OV) return "VOUT_OV";
    if (fault == PWR_FAULT_IOUT_OC) return "IOUT_OC";
    return "multiple";
}

float Power_GetVinGain(void)
{
    return s_vin_gain;
}

float Power_GetVoutGain(void)
{
    return s_vout_gain;
}

float Power_GetIoutGain(void)
{
    return s_iout_gain;
}

void Power_EmergencyStop(void)
{
    if (s_hrtim_ready != 0u)
    {
        HRTIM1->sCommonRegs.ODISR = HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2;
    }
}
