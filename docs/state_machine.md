# Finite State Machine

This document summarizes the converter state machine implemented in the digital power control (DPC) library. The states are defined in [`Drivers/BSP/DPC_FW_PACK/inc/DPC_FSM.h`](../Drivers/BSP/DPC_FW_PACK/inc/DPC_FSM.h) as the `DPC_FSM_State_t` enumeration.

## States

- **`DPC_FSM_WAIT`** - persistent wait state after clearing faults. Normally transitions to `IDLE` once the wait time expires.
- **`DPC_FSM_IDLE`** - idle state when input conditions are valid. From here the machine enters `INIT` or `STOP` on a fault.
- **`DPC_FSM_INIT`** - short initialization step between `IDLE` and `START`. Initializes control variables.
- **`DPC_FSM_START`** - performs the converter start-up. When the bus reference voltage is reached the next state is `RUN`.
- **`DPC_FSM_RUN`** - normal running state. Faults or stop commands move the machine to `STOP`.
- **`DPC_FSM_STOP`** - pass-through state that disables PWM outputs. Subsequent state is `ERROR` or `FAULT` depending on the condition.
- **`DPC_FSM_ERROR`** - indicates an invalid state or recoverable error. Once cleared the machine goes back to `WAIT`.
- **`DPC_FSM_FAULT`** - persistent state entered after a fault. Recovery typically requires a system reset.

## Typical Transitions

A typical power-up sequence progresses through the following states:

```
IDLE -> INIT -> START -> RUN
```

Fault or stop conditions can move the machine to `STOP` or directly to `FAULT` at any time. After handling the condition, the machine returns to `WAIT` and eventually `IDLE`.

### Simplified Diagram

```
WAIT -> IDLE -> INIT -> START -> RUN
  ^                               |
  |                               v
 ERROR <------- STOP <--------- FAULT
```

This diagram shows that from `RUN` the machine may transition to `STOP` (e.g. on a fault). Recoverable errors eventually lead back to `WAIT` whereas unrecoverable faults remain in `FAULT` until a reset.

