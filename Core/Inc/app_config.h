/*
 * app_config.h
 *
 * Board-specific configuration for the DPOS G4 BUCK V2 digital power stage
 * (STM32G474CCU6, 12-48 V input, 22 uH inductor, 200 kHz switching).
 *
 * Pin map (from the vendor schematic):
 *   PA1  ADC2_IN2   Iout_detc   (0.03 R shunt, ~525 mV/A)
 *   PA6  ADC2_IN3   Vout_detc   (33k/2k divider, 17.5:1)
 *   PA7  ADC2_IN4   Vin_detc    (33k/2k divider, 17.5:1)
 *   PA8  HRTIM_CHA1 high-side gate
 *   PA9  HRTIM_CHA2 low-side gate
 *   PA2  USART2_TX  shell
 *   PA3  USART2_RX  shell
 *   PB0/PB1/PB2     red/green/blue LED, active low
 *
 * USB, SPI2 and I2C are not used; those pins stay unconfigured.
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* -------------------------------------------------------------------------- */
/* Firmware identification                                                     */
/* -------------------------------------------------------------------------- */
#define FW_NAME             "DPOS-G474-BUCK"
#define FW_VERSION          "1.0.0"
#define FW_BOARD            "STM32G474CCU6 DPOS G4 BUCK V2"
#define FW_BUILD_DATE       __DATE__ " " __TIME__

/* -------------------------------------------------------------------------- */
/* Power stage timing                                                          */
/* -------------------------------------------------------------------------- */
#define PWM_PERIOD_COUNTS   27200u      /* 200 kHz with HRTIM x32 prescaler @170 MHz */
#define PWM_FREQ_HZ         200000u
#define CONTROL_FREQ_HZ     100000u     /* HRTIM repetition = 1 -> every 2 periods  */

/* -------------------------------------------------------------------------- */
/* Defaults and limits (milli-units)                                           */
/* -------------------------------------------------------------------------- */
#define DEFAULT_VSET_MV     5000u
#define DEFAULT_ILIM_MA     3000u
#define VSET_MIN_MV         1000u
#define VSET_MAX_MV         40000u
#define ILIM_MAX_MA         5000u       /* 22 uH / 5 A inductor rating */

/* -------------------------------------------------------------------------- */
/* Protection thresholds                                                       */
/* -------------------------------------------------------------------------- */
#define VIN_UV_MV           10000u
#define VIN_OV_MV           50000u
#define VOUT_OV_MARGIN_MV   1500u
#define VOUT_OV_ABS_MV      40000u
#define IOC_MARGIN_MA       500u
#define IOC_ABS_MA          6000u
#define IOC_FAST_MA         6000u   /* hard-short trip, no debounce */
#define IOC_DEBOUNCE_MS     2u      /* sustained limit+margin must last this long */
#define IOC_DEBOUNCE_TICKS  ((IOC_DEBOUNCE_MS * CONTROL_FREQ_HZ) / 1000u)

/* -------------------------------------------------------------------------- */
/* Loop shaping                                                                */
/* -------------------------------------------------------------------------- */
#define VREF_RAMP_MV_TICK   1.0f        /* 1 mV / 10 us -> 100 V/s soft start      */
#define DUTY_MIN            0.005f
#define DUTY_MAX            0.850f

/* Incremental PI(D) gains, same family as the vendor reference examples.
 * Errors are expressed in volts / amps, the integrator state is in "Vosc"
 * units where 3.3 corresponds to 100 % duty. */
#define KP_V                0.10f
#define KI_V                0.001f
#define KD_V                0.50f
#define KP_I                0.11f
#define KI_I                0.001f
#define KD_I                0.50f
#define VOSC                3.3f
#define OSC_LIMIT           3.036f      /* ~92 % duty clamp in Vosc units */

/* -------------------------------------------------------------------------- */
/* ADC scaling                                                                 */
/* -------------------------------------------------------------------------- */
#define ADC_VREF_MV         3300.0f
#define ADC_FULL_SCALE      4096.0f
#define VIN_DIVIDER         17.5f
#define VOUT_DIVIDER        17.5f
#define IOUT_MV_PER_AMP     525.0f      /* vendor reference: 0.0019047 A/mV */

/* Calibration offsets, measured with a DMM at zero output. */
#define CAL_VOUT_OFFSET_MV  0
#define CAL_IOUT_OFFSET_MA  0

/* Calibration gains, fixed at build time.  Adjust and rebuild if the sense
 * divider or board revision changes. */
#define CAL_VIN_GAIN        0.985f   /* measured 15.269 V vs 15.05 V DMM */
#define CAL_VOUT_GAIN       0.94f   /* measured 5.00 V display vs 4.70 V DMM */
#define CAL_IOUT_GAIN       1.0f

/* Readings below this are shown as zero: the current-sense amplifier offset
 * and ADC quantization are about 1-2 LSB (~2-3 mA).  Set to 0 to disable. */
#define IOUT_ZERO_DEADBAND_MA 10u

#endif /* APP_CONFIG_H */
