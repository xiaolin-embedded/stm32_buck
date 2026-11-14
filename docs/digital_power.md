# Digital power core — control, CV/CC and tuning

This document explains the actual control code in `Core/Src/app_power.c` and
how to tune it.  The hardware is a 200 kHz synchronous buck on the DPOS G4
BUCK V2 board (STM32G474CCU6, 22 µH / 5 A, 12–48 V input).

## 1. Power stage and timing

| Item | Value |
|---|---|
| PWM period | 27 200 HRTIM counts = 5 µs = **200 kHz** |
| Control tick | HRTIM repetition = 1 → **100 kHz** (every 2 PWM periods) |
| Dead time | 20 counts at 170 MHz = **118 ns**, rising and falling |
| Outputs | TA1 = PA8 high side, TA2 = PA9 low side (complementary) |
| ADC trigger | Timer A CMP3 = CMP1 / 2, i.e. the middle of the on-time |
| Duty range | 0.5 % … 85 % (software clamp) |

The HRTIM counter runs continuously.  The output is set on the timer period
event and reset on CMP1; the dead-time unit generates the complementary
low-side signal.  CMP1/CMP3 are preloaded and applied on the master period.

## 2. Sensing and ADC

ADC2 scans three channels on every HRTIM trigger:

| Rank | Pin | Signal | Conversion |
|---|---|---|---|
| 1 | PA6 | Vout | `raw * 3300 / 4096 * 17.5 * CAL_VOUT_GAIN` |
| 2 | PA1 | Iout | `(raw * 3300 / 4096) / 525 mV/A * CAL_IOUT_GAIN` |
| 3 | PA7 | Vin | `raw * 3300 / 4096 * 17.5 * CAL_VIN_GAIN` |

- DMA1 channel 3, circular, 3 half-words; the control ISR reads each value
  once.
- A first-order IIR (`3/4 old + 1/4 new`) filters each channel.  The filter
  history is reset only in `Power_Init()`; it must not be reset in the
  OFF/FAULT states or the readings will be divided by four.
- `CAL_VIN_GAIN = 0.985`, `CAL_VOUT_GAIN = 0.94`, `CAL_IOUT_GAIN = 1.0` are
  the values measured on this board.  They are compile-time constants in
  `Core/Inc/app_config.h`; change them there and rebuild.
- Currents below `IOUT_ZERO_DEADBAND_MA` (10 mA) are reported as zero.  At
  525 mV/A one ADC LSB is about 1.53 mA, so this hides the amplifier offset
  and quantization.

## 3. Reference ramp and soft start

`s_vref_mv` is the active voltage reference.  It moves toward `vset` at
1 mV per control tick = **100 V/s**:

- start: 0 → vset (soft start);
- increase: ramp up;
- decrease: ramp down at the same rate (5 V → 1.8 V takes about 32 ms).

The output OV limit uses `max(vref, vset) + 1.5 V`, capped at 40 V.  This is
important: if the limit used the new, lower `vset` immediately, the still-high
output would trip VOUT_OV on a normal downward setpoint change.

## 4. Voltage loop

Incremental PI(D) in error form, error in volts:

```
e_v(k)  = (vref - vout) * 0.001
de_v(k) = e_v(k) - e_v(k-1)
du_v   += Kp_V*de_v + Ki_V*e_v + Kd_V*(de_v - de_v(k-1))
u_v     = clamp(du_v, 0, 3.036)          // Vosc units, 3.3 = 100 %
duty_v  = u_v / 3.3
```

Default gains: `Kp_V = 0.10`, `Ki_V = 0.001`, `Kd_V = 0.50`.
`3.036 / 3.3 = 92 %` is the command clamp; the final duty clamp is 85 %.

## 5. Current loop

The inner loop is an average-current PI(D) with error in amps:

```
e_i(k)  = (ilim - iout) * 0.001
de_i(k) = e_i(k) - e_i(k-1)
du_i   += Kp_I*de_i + Ki_I*e_i + Kd_I*(de_i - de_i(k-1))
u_i     = clamp(du_i, 0, 3.036)
duty_i  = u_i / 3.3
```

Default gains: `Kp_I = 0.11`, `Ki_I = 0.001`, `Kd_I = 0.50`.

Both loops run every 10 µs.  The applied duty is:

```
duty = min(duty_v, duty_i)
```

so the voltage loop controls in CV and the current loop takes over in CC.
When the current loop is dormant (CV), its integrator stays at the clamp;
when the load increases, it leaves the clamp and takes over.

### CV/CC detection

A status flag is derived from the same arbitration, with hysteresis:

```
enter CC: du_i + 0.02 < du_v
leave CC: du_i > du_v + 0.05
```

Only valid in `RUN`; `SOFT_START`, `OFF` and `FAULT` report CV.  The flag is
published in telemetry and shown as `RUN (CV)` / `RUN (CC)` in `status`, and
`/CV` / `/CC` in `monitor`.

### Load-type behaviour

With `vset = 5 V`, `ilim = 1 A`:

| Load | Mode | Vout | Iout |
|---|---|---|---|
| 10 Ω | CV | 5 V | 0.5 A |
| 5 Ω | boundary | 5 V | 1 A |
| 2 Ω | CC | 2 V | 1 A |
| short | CC | ≈0 V | 1 A |

A constant-current electronic load set above `ilim` forces the supply into
CC; the output voltage collapses to whatever the load's compliance allows.
That is expected: the supply is a current-limited voltage source.

## 6. Protection

| Fault | Threshold | Action |
|---|---|---|
| VIN_UV | Vin < 10 V | latch, outputs off |
| VIN_OV | Vin > 50 V | latch, outputs off |
| VOUT_OV | Vout > min(max(vref, vset) + 1.5 V, 40 V) | latch, outputs off |
| IOUT_OC (sustained) | Iout > ilim + 0.5 A for 2 ms | latch, outputs off |
| IOUT_OC (software fast) | filtered Iout > 6 A immediately | latch, outputs off |
| PEAK_OC (hardware) | COMP3: I_PEAK_DETC > DAC1 threshold (≈7 A) | latch, outputs off in µs |

- Outputs are disabled immediately through `HRTIM1->sCommonRegs.ODISR`.
- Faults latch; `reset` stops the output and clears them.  A real fault must
  be removed before restarting.
- The 2 ms debounce is deliberate: a load step into CC briefly exceeds
  `ilim + 0.5 A` while the current loop takes over.  The filtered 6 A trip and
  the hardware peak comparator protect against a hard short or a failed
  current loop.
- **Hardware peak comparator**: U7A amplifies `I_PEAK_DETC` by about 7.4× over
  the 0.03 Ω shunt, i.e. ≈222 mV/A.  DAC1_CH1 sets the threshold (≈7 A →
  code ≈1930).  COMP3 compares PA0 against the DAC; a rising edge triggers
  `COMP1_2_3_IRQHandler`, which sets `PEAK_OC` and disables the outputs
  immediately through `ODISR`.  This path is independent of the ADC filter
  and the 100 kHz loop.  `Power_Start()` refuses to start if the comparator
  output is still high, so a persistent short cannot be re-enabled by simply
  clearing the fault.
- `Power_EmergencyStop()` is called from `Error_Handler`, the hard/memory
  fault handlers and the FreeRTOS hooks so a firmware failure does not leave
  the bridge switching.

## 7. ISR budget and telemetry

- The control ISR runs at 100 kHz (10 µs budget).  `app_power.c` is compiled
  with `-O2` even in Debug; the measured code is well inside the budget.
- No HAL or FreeRTOS calls are made in the control ISR.
- Telemetry crosses to the shell task through a seqlock (`s_seq` odd/even),
  so `status`/`monitor` always get a consistent snapshot.
- USART2 RX is interrupt-driven into a 256-byte ring buffer; the shell task
  polls every 10 ms.

## 8. Tuning procedure

### Equipment

- Current-limited bench supply, 12–48 V.
- Dummy load: four 5 Ω / 2 W resistors in 2-series-2-parallel = 5 Ω / 8 W,
  giving 1 A at 5 V; or an electronic load.
- DMM and, preferably, an oscilloscope (SW node, gate drives).

### Before the first power-up

1. Set `vset` low (e.g. 3300 mV) and `iset` low (e.g. 500 mA).
2. `poweron`, verify `RUN/CV`, Vout and the DMM agree after the calibrated
   gains.
3. Only then increase voltage/current.

### Voltage loop

- Start with the vendor gains; change one gain at a time.
- Apply a load step that stays below the current limit (e.g. 10 Ω → 5 Ω at
  5 V) and watch Vout with a scope:
  - too much overshoot/ringing: reduce `Kp_V` and `Kd_V`;
  - slow recovery: increase `Ki_V` slightly;
  - steady-state error: `Ki_V` is the integral term.
- The incremental form means `Kp` acts on the error difference, `Ki` on the
  error, and `Kd` on the second difference; large `Kd` can amplify noise.

### Current loop

- Set `iset` below what the load would draw (e.g. 1 A into 2 Ω at 5 V).
- The supply should enter `RUN/CC`, Vout = Ilim × R, and stay stable.
- If CC is slow to take over, increase `Kp_I`/`Ki_I` slightly; if it
  oscillates, reduce them.
- The current loop is average current at 100 kHz.  A hard short is handled by
  the 6 A software fast trip and the ≈7 A hardware COMP3/DAC comparator; the
  hardware path is the fast one.

### Load steps into CC

- A large step (e.g. 10 Ω → 2 Ω) briefly exceeds `ilim + 0.5 A`; the 2 ms
  debounce lets the current loop settle without a false fault.
- A step that produces more than 6 A (e.g. 10 Ω → 1 Ω at 5 V) trips the fast
  protection; the ≈7 A hardware comparator may catch it first.  This is
  intended, because the 5 A inductor is being exceeded.

### Calibration

- `CAL_VIN_GAIN` / `CAL_VOUT_GAIN` / `CAL_IOUT_GAIN` are fixed in
  `app_config.h`.  Measure with a DMM/known load, adjust, rebuild.
- The nominal divider is 17.5:1; this board needed about 0.94 on Vout
  (≈16.45:1) and 0.985 on Vin.
- Keep the 10 mA current deadband unless measuring mA-level currents.

## 9. Common pitfalls found on this board

- `HAL_HRTIM_MspPostInit()` is not called by
  `HAL_HRTIM_WaveformOutputConfig()` in this HAL version.  Without the
  explicit call, PA8/PA9 stay inputs and the gate driver receives no PWM.
- The high-resolution ×32 prescaler needs the HRTIM DLL calibration
  (`HAL_HRTIM_DLLCalibrationStart` / `PollForDLLCalibration`).
- Do not reset the ADC moving-average history in OFF/FAULT; it divides the
  readings by the filter length.
- A downward `vset` change must ramp, and the OV limit must follow
  `max(vref, vset)`, otherwise it false-trips VOUT_OV.
- A load step into CC needs the OC debounce; otherwise the transient trips
  IOUT_OC.  The hardware peak comparator threshold must sit above the normal
  ripple and below the inductor/switch stress; ≈7 A on this board.
- The input divider and Vout divider are not exact; calibrate or bake in
  gains.
- At no load/output off, a ~0.3 V residual can come from high-side MOSFET
  leakage; a 1 kΩ–10 kΩ output bleeder removes it.  It is harmless with a
  normal load.

## 10. Limits and test checklist

- 5 A is the inductor limit; continuous power is thermal-limited.  The
  vendor examples use 5 A at 10 V and 3 A at 24–28 V.
- Buck only: Vout < Vin.  Firmware output range 1–40 V.
- Average-current limit, not a hardware peak-current comparator; the
  COMP3/DAC circuit is unused.
- No reverse-polarity protection.

Checklist:

- [ ] No load: Vout matches DMM, `RUN/CV`, blue heartbeat.
- [ ] 5 Ω / 1 A: `RUN/CV`, Pout ≈ 5 W.
- [ ] `iset 500` into 5 Ω: `RUN/CC`, Vout ≈ 2.5 V, Iout ≈ 0.5 A.
- [ ] Load step 10 Ω ↔ 2 Ω: no false fault; CC recovers.
- [ ] `vset 1800` while running: ramps down, no VOUT_OV.
- [ ] `poweroff`: green off, blue heartbeat, output ≈ 0 with a bleeder/load.
- [ ] Fault test (e.g. input below UV): red, `reset` clears.
- [ ] Peak comparator: with a current-limited supply/electronic load, raise the
  current above ~7 A; `PEAK_OC` should latch within microseconds.  If the
  threshold is off, trim `IOC_PEAK_MA` / `IOC_PEAK_MV_PER_A`.
