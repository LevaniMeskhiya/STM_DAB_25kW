# Configuration Parameters

This file summarizes key configuration macros defined in
`DPC_DAB/STMicroelectronics.DPC_DAB_conf.h`.
These constants tune the digital power control (DPC) library for the dual-active-bridge converter.

## Voltage and Current Scaling

The ADC scaling coefficients convert raw counts into real voltages or currents. They
are used inside the middleware to compute physical values from the sensed inputs.

| Macro | Purpose |
|-------|---------|
| `eDS_DPC_DAB_G_VDC1` | Gain for input voltage measurement on the high-voltage side |
| `eDS_DPC_DAB_B_VDC1` | Offset for the same high-voltage measurement |
| `eDS_DPC_DAB_G_IDC1` | Gain for input current sensing |
| `eDS_DPC_DAB_B_IDC1` | Offset for input current sensing |
| `eDS_DPC_DAB_G_VDC2` | Gain for output voltage measurement on the low-voltage side |
| `eDS_DPC_DAB_B_VDC2` | Offset for the low-voltage measurement |
| `eDS_DPC_DAB_G_IDC2` | Gain for output current sensing |
| `eDS_DPC_DAB_B_IDC2` | Offset for output current sensing |

These coefficients directly determine the scaling of voltage and current
variables throughout the control loop.

## PI Controller Gains

The voltage and current controllers use PI compensators. The macros below set
the proportional (`KP`) and integral (`KI`) terms, along with optional
anti-windup features.

| Macro | Description |
|-------|-------------|
| `eDS_DPC_DAB_VCTRL_KP` | Proportional gain for the voltage loop |
| `eDS_DPC_DAB_VCTRL_KI` | Integral gain for the voltage loop |
| `eDS_DPC_DAB_VCTRL_PI_SAT_EN` | Enable reference saturation for the voltage controller |
| `eDS_DPC_DAB_VCTRL_PI_AW_EN` | Enable anti-windup logic for the voltage controller |
| `eDS_DPC_DAB_VCTRL_PI_SAT_UP` | Upper saturation limit for the voltage controller |
| `eDS_DPC_DAB_VCTRL_PI_SAT_DOWN` | Lower saturation limit for the voltage controller |
| `eDS_DPC_DAB_ICTRL_KP` | Proportional gain for the current loop |
| `eDS_DPC_DAB_ICTRL_KI` | Integral gain for the current loop |
| `eDS_DPC_DAB_ICTRL_PI_SAT_EN` | Enable reference saturation for the current controller |
| `eDS_DPC_DAB_ICTRL_PI_AW_EN` | Enable anti-windup logic for the current controller |
| `eDS_DPC_DAB_ICTRL_PI_SAT_UP` | Upper saturation limit for the current controller |

Tuning these gains changes the dynamic response of the DAB converter.

## PWM Timing

PWM related macros set the switching frequency and dead-time values used by the
HRTIM peripheral.

| Macro | Description |
|-------|-------------|
| `eDS_DPC_DAB_PWM_FREQ` | Switching frequency in hertz |
| `eDS_DPC_DAB_DT_DAB1` | Dead time for phase-leg 1 (seconds) |
| `eDS_DPC_DAB_DT_DAB2` | Dead time for phase-leg 2 (seconds) |
| `eDS_DPC_DAB_PWM_INIT` | Initial PWM mode (armed or safe) |

These parameters control the timing of the power stage and must match the
hardware design.

## Protection Thresholds

Several macros define over-voltage, under-voltage and over-current thresholds.
When any of these limits are exceeded, the middleware will enter a fault state
and disable the converter.

| Macro | Threshold |
|-------|-----------|
| `eDS_DPC_DAB_VDCHV_OVP` | High-voltage over-voltage protection (volts) |
| `eDS_DPC_DAB_VDCHV_UV` | High-voltage under-voltage limit (volts) |
| `eDS_DPC_DAB_VDCHV_UVLO` | High-voltage lockout level (volts) |
| `eDS_DPC_DAB_IDCHV_OCP` | High-voltage side over-current (amps) |
| `eDS_DPC_DAB_IDCLV_OCP` | Low-voltage side over-current (amps) |
| `eDS_DPC_DAB_VDCLV_OVP` | Low-voltage over-voltage protection (volts) |
| `eDS_DPC_DAB_VDCHV_MIN` | Minimum allowed high-voltage value (startup) |

## Other Parameters

Additional macros cover the transformer turn ratio, inductance value and the
initial DAC settings used for debugging.

- `eDS_DPC_DAB_INDUCTANCE` - external inductor value in henries
- `eDS_DPC_DAB_TRAFO_TURN_RATIO` - transformer ratio between the two bridges
- `eDS_DPC_DAB_DAC_CH1_INIT`, `eDS_DPC_DAB_DAC_CH2_INIT`, `eDS_DPC_DAB_DAC_CH3_INIT` - default signals on the debug DAC channels

These constants provide basic hardware description and initial conditions for
the firmware.

---
The combination of scaling coefficients, controller gains and PWM timing
parameters governs how the STM32 interprets feedback signals and drives the
power stage. Protection limits safeguard the converter against fault
conditions.
