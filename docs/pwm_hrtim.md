# HRTIM PWM Control

This file details how the high-resolution timer (HRTIM) is used to generate the dual-active-bridge PWM waveforms. The implementation relies exclusively on HRTIM1 and maps each half-bridge leg to one of the timer outputs.

## Timer Mapping

The mapping of channels and timer indexes is defined in `DPC_DAB/App/DPC_Lib_Conf.h`:

```c
#define PWM_Tim1                        hhrtim1
#define PWM_CHANNEL_1                   HRTIM_OUTPUT_TA1
#define PWM_CHANNEL_1N                  HRTIM_OUTPUT_TA2
#define PWM_CHANNEL_2                   HRTIM_OUTPUT_TB1
#define PWM_CHANNEL_2N                  HRTIM_OUTPUT_TB2
#define PWM_CHANNEL_3                   HRTIM_OUTPUT_TC1
#define PWM_CHANNEL_3N                  HRTIM_OUTPUT_TC2
#define PWM_CHANNEL_4                   HRTIM_OUTPUT_TD1
#define PWM_CHANNEL_4N                  HRTIM_OUTPUT_TD2
#define PWM_IDX_CH_1                    HRTIM_TIMERINDEX_TIMER_A
#define PWM_IDX_CH_2                    HRTIM_TIMERINDEX_TIMER_B
#define PWM_IDX_CH_3                    HRTIM_TIMERINDEX_TIMER_C
#define PWM_IDX_CH_4                    HRTIM_TIMERINDEX_TIMER_D
#define PWM_IDX_CH_5                    HRTIM_TIMERINDEX_MASTER
```

Each leg of the converter therefore uses one timer (A-D) with complementary outputs, while the master timer provides synchronisation and phase-shift reference.

## Initialization and Control Functions

PWM configuration and runtime control are performed by the following functions in `DPC_Actuator.c`:

- `DPC_ACT_Init()` - computes the timer period, programs compare registers and selects the modulation scheme.
- `DPC_ACT_HRTIM_Start()` - starts the master and leg timers and enables the waveforms.
- `DPC_ACT_HRTIM_OutEnable()` / `DPC_ACT_HRTIM_OutDisable()` - turn the HRTIM outputs on or off.
- `DPC_ACT_OutEnable()` / `DPC_ACT_OutDisable()` - high level wrappers used by the application state machine.
- `DPC_ACT_Calc_DeadTime()` - converts a desired dead-time in seconds into the best DTG register value.
- `DPC_ACT_DAB_Conf_DeadTime()` - writes the calculated dead-time to all HRTIM timers.
- `DPC_ACT_CPWM_PS_Update()` - updates the phase-shift for single phase-shift modulation (conventional PWM).
- `DPC_ACT_DAB_TpzPWM_PS_Update()` - updates all compare units for trapezoidal PWM.

The core of the dead-time computation is shown below:

```c
uint8_t DPC_ACT_Calc_DeadTime(float DT_TimeVal)
{
  ...
  DTG_bin=(uint32_t)(DT_TimeVal/T_DTG);
  return DTG_bin;
}
```

Phase-shift updates for trapezoidal modulation compute timer ticks and program compare units on the master timer:

```c
void DPC_ACT_DAB_TpzPWM_PS_Update(DPC_ACT_PWM_t *tDPC_PWM_loc,
                                  float D1_Duty, float D2_Duty, float PS_Duty)
{
  ...
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_5 ,HRTIM_COMPAREUNIT_1, (uint32_t)(HB2_Timer_Tick));
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_5 ,HRTIM_COMPAREUNIT_2, (uint32_t)(HB3_Timer_Tick));
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_5 ,HRTIM_COMPAREUNIT_3, (uint32_t)(HB4_Timer_Tick));
}
```

For conventional single phase-shift PWM the compare update is simpler:

```c
void DPC_ACT_CPWM_PS_Update(DPC_ACT_PWM_t *tDPC_PWM_loc, float PS_Duty)
{
  ...
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_5 ,HRTIM_COMPAREUNIT_1, (uint32_t)(PS_Timer_Tick));
}
```

## Modulation Techniques

The control library supports several modulation modes, selected by the `ModTecnique` field of `DPC_LCT_DAB_Ctrl_t`:

- **SPWM** - single phase-shift modulation.
- **TPZ** - trapezoidal PWM (used for adaptive phase shift).
- **TRG** - triangular modulation.
- **MAN** - manual duty/phase entry for debugging.

`DPC_LPCNTRL_DAB_ModulatorSelector()` chooses the active technique and provides the normalised duty and phase values to the actuator layer.

## Phase Calculation

`DPC_LPCNTRL_DAB_TpzPWM_DutyTime_Calc()` computes the required phase and duty values based on input/output voltages, transformer ratio and desired phase angle. The resulting duty factors are passed to `DPC_ACT_DAB_TpzPWM_PS_Update()` for application to the timer registers.

## HRTIM Structure

- **Master Timer** - sets the overall PWM period and triggers compare events for phase shifting and ADC sampling.
- **Timer A/B** - drive the high-voltage bridge legs using outputs `TA1/TA2` and `TB1/TB2`.
- **Timer C/D** - drive the low-voltage bridge legs using outputs `TC1/TC2` and `TD1/TD2`.

The timers run in continuous up-counting mode and use the master compare events to reset their counters, realising the desired phase shift between primary and secondary bridges.


## Register Configuration

`MX_HRTIM1_Init()` in `Core/Src/hrtim.c` programs the master and all four slave timers. The master timer defines the base period and generates compare events used to phase shift the legs.

### Counter Reset Sources

| Timer | Reset trigger |
|-------|---------------|
| **Timer A** | Master period (`HRTIM_TIMRESETTRIGGER_MASTER_PER`) |
| **Timer B** | Master compare 1 (`HRTIM_TIMRESETTRIGGER_MASTER_CMP1`) |
| **Timer C** | Master compare 2 (`HRTIM_TIMRESETTRIGGER_MASTER_CMP2`) |
| **Timer D** | Master compare 3 (`HRTIM_TIMRESETTRIGGER_MASTER_CMP3`) |

### Output Set/Reset Sources

The high-side outputs use Channel 1 of each timer. Their events are configured as:

```
Timer A: SetSource = MASTERPER | TIMPER,  ResetSource = TIMCMP1
Timer B: SetSource = MASTERCMP1 | TIMPER, ResetSource = TIMCMP1
Timer C: SetSource = MASTERCMP2 | TIMPER, ResetSource = TIMCMP1
Timer D: SetSource = MASTERCMP3 | TIMPER, ResetSource = TIMCMP1
```

Complementary channels (TA2-TD2) have `SetSource = NONE` and `ResetSource = NONE` so they simply mirror the main channel with dead-time insertion.

### ADC Trigger

The master compare 4 event (`HRTIM_ADCTRIGGEREVENT13_MASTER_CMP4`) triggers ADC conversions to synchronise sampling with the PWM period.