# Data Structures

This page summarizes important structures and enumerations used by the firmware.

## Application Structures

### `DPC_DAB_t`
Defined in [`DPC_Application.h`](../DPC_DAB/App/DPC_Application.h). It aggregates
all state and configuration required by the DAB controller. Key members include:

- `INRUSH_CTRL` – [`DPC_LCT_InrushCtrl_t`] inrush current controller.
- `BURST_CTRL` – [`DPC_ACT_Burst_t`] burst mode handler.
- `pDAB_CTRL` – [`DPC_LCT_DAB_Ctrl_t`] main DAB control block.
- `tDPC_PWM` – [`DPC_ACT_PWM_t`] actuator parameters for the PWM generators.
- ADC and status data such as `DAB_ADC_RAW`, `DAB_ADC_PHY`, and DC source/load
  structures.

#### Enumerations

The same header also defines the run‑time state machines used by the
application:

- `DAB_FSM_State_TypeDef` – main finite state machine for the converter.
- `Run_State_TypeDef` – run controller state.
- `DPC_DAB_InitMode_t` – initial operating mode selection.

## Common Data Package

`Drivers/BSP/DPC_FW_PACK/inc/DPC_CommonData.h` contains many helper structs.
Representative examples are listed below.

| Struct | Purpose |
|-------|---------|
| `DPC_LCT_VDC_Ctrl_t` | DC‑bus voltage control handler holding the PI regulator for Vdc. |
| `DPC_PI_t` | Generic PI regulator structure used by the control loops. |
| `DAB_ADC_PHY_Struct_t` | Scaled ADC readings of DAB voltages and currents. |
| `DPC_LCT_InrushCtrl_t` | Parameters and status for the inrush current routine. |
| `DPC_ACT_PWM_t` | Dynamic PWM configuration and status. |
| `DPC_ACT_Burst_t` | Burst mode thresholds and status information. |
| `DPC_MISC_DCLoadLimit_t` | Current and voltage limits for the low‑voltage load. |
| `DPC_MISC_DCSourceLimit_t` | Thresholds for the high‑voltage source. |
| `DPC_MISC_DCSource_t` | Instantaneous values and status of the HV source. |
| `DPC_MISC_DCLoad_t` | Current load status information. |
| `DAC_Channel_STRUCT` | Mapping of DAC output channels with gain and bias settings. |

