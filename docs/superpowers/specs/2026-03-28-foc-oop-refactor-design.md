# FOC OOP Refactor Design Spec

## Overview

Refactor the FOC core class and App layer for proper OOP encapsulation while maintaining zero-overhead and ISR safety.

**Constraints:**
- No virtual functions, no dynamic allocation (new/delete)
- Getter/setter in headers for guaranteed inlining
- Only modify `User/` directory
- Preserve ISR execution efficiency (20kHz current loop)
- Incremental: compile-verify after each step

## Scope

### Delete
- `ControlMode::RATCHET` and `ControlMode::INERTIA` enum values
- All ratchet params: `ratchetSpacing`, `ratchetKp`, `ratchetKd`
- All inertia params: `inertiaDecay`, `inertiaMaxCurrent`, `inertiaKp`, `inertiaVelocityThreshold`
- Corresponding case branches in `MotorControlTask()`
- Inertia loop logic in `MotorControlTask()`

### Keep
- `ControlMode`: `NONE`, `TORQUE`, `VELOCITY`, `POSITION`, `MIT`
- `MotorMode`: `INIT`, `NONE`, `RUNNING`

## Design

### 1. PID: Keep Pure C + Add C++ Wrapper

`pid.h` / `pid.c` remain unchanged (pure C).

New lightweight C++ wrapper class `PidController` defined in `Foc.h`:

```cpp
class PidController {
private:
    PIDControllerTypedef pid_;
public:
    PidController() = default;

    void Init(float kp, float ki, float kd, float sampleTime,
              float outputLimits, float integralLimits, float deadBand,
              PIDModeTypedef mode) {
        PID_Init(&pid_, kp, ki, kd, sampleTime, outputLimits, integralLimits, deadBand, mode);
    }

    float Compute(float current, float target) { return PIDCompute(&pid_, current, target); }
    void SetOutputLimits(float min, float max) { PID_SetOutputLimits(&pid_, min, max); }
    void Reset() { PID_Reset(&pid_); }

    // VOFA pointer access for online tuning
    float* KpPtr() { return &pid_.Kp; }
    float* KiPtr() { return &pid_.Ki; }
    float* KdPtr() { return &pid_.Kd; }

    // Read-only access
    float Integral() const { return pid_.integral; }
    void ClearIntegral() { pid_.integral = 0.0f; }
};
```

### 2. FOC Class Member Reclassification

#### Private + inline getter (read-only state)

| Member | Getter | Notes |
|--------|--------|-------|
| `Ia_`, `Ib_`, `Ic_` | `float Ia() const` | Phase currents |
| `Iq_`, `Id_` | `float Iq() const` | dq currents |
| `motorAngle_` | `float MotorAngle() const` | Mechanical angle |
| `angleSingleTurn_` | `float AngleSingleTurn() const` | Single-turn angle |
| `velocity_` | `float Velocity() const` | Angular velocity |
| `uAlpha_`, `uBeta_` | None | Internal only |
| `voltageA/B/C_`, `*Duty_` | None | Internal only |
| `currentA/B/COffset_` | None | Internal only |
| `Vbus_` | None | Internal only |
| `iqSetpoint_`, `idSetpoint_`, `velocitySetpoint_`, `positionSetpoint_` | None | Internal control |

#### Private + getter + VOFA pointer (tunable targets)

| Member | Methods |
|--------|---------|
| `targetPosition_` | `float TargetPosition() const`, `float* TargetPositionPtr()` |
| `targetSpeed_` | `float TargetSpeed() const`, `float* TargetSpeedPtr()` |
| `targetCurrent_` | `float TargetCurrent() const`, `float* TargetCurrentPtr()` |
| `targetTorque_` | `float TargetTorque() const`, `float* TargetTorquePtr()` |

#### Private + controlled interface (state machine)

| Member | Interface |
|--------|-----------|
| `controlMode_` | `ControlMode GetControlMode() const`; set implicitly via `SetTorqueTarget()` etc. |
| `motorMode_` | Fully internal; managed by `Tick()` state machine |

#### PID controllers (private, exposed via reference for VOFA)

| Member | Access |
|--------|--------|
| `speedPid_` | `PidController& SpeedPid()` |
| `positionPid_` | `PidController& PositionPid()` |
| `currentPid_` | `PidController& CurrentPid()` |

### 3. Unified Target-Setting API

Replace direct member writes + controlMode assignment with:

```cpp
void SetTorqueTarget(float torque);    // sets targetCurrent_, controlMode_ = TORQUE
void SetVelocityTarget(float speed);   // sets targetSpeed_, controlMode_ = VELOCITY
void SetPositionTarget(float pos);     // sets targetPosition_, controlMode_ = POSITION
void SetMitTarget(float pos, float vel, float torque, float kp, float kd);  // controlMode_ = MIT
void Stop();                           // controlMode_ = NONE, clear setpoints
```

These methods handle any necessary state cleanup on mode switch (e.g., clearing PID integrals).

### 4. State Machine Internalization

Move the MotorMode state machine from `MotorTask()` into `FOC::Tick()`:

```cpp
void FOC::Tick() {
    UpdateCurrent();
    UpdateMotorAngle();
    // Velocity at 1kHz (every 20 ticks)
    loopCount_++;
    if (loopCount_ >= 20) {
        loopCount_ = 0;
        UpdateVelocity();
    }
    // Motor mode state machine
    switch (motorMode_) {
    case MotorMode::INIT:
        motorMode_ = MotorMode::NONE;
        break;
    case MotorMode::NONE:
        motorMode_ = MotorMode::RUNNING;
        targetSpeed_ = 0.0f;
        velocitySetpoint_ = 0.0f;
        controlMode_ = ControlMode::TORQUE;
        targetCurrent_ = 0.0f;
        break;
    case MotorMode::RUNNING:
        MotorControlTask();
        break;
    }
}
```

Note: default startup mode changed from INERTIA to TORQUE with zero target (motor free).

### 5. File Changes Summary

| File | Change |
|------|--------|
| `User/Lib/Inc/Foc.h` | Add PidController wrapper; reclassify FOC members; add getters/setters; remove ratchet/inertia params |
| `User/Lib/Src/Foc.cpp` | Implement new methods; delete ratchet/inertia logic; add Tick(); update MotorControlTask() |
| `User/App/Src/MotorTask.cpp` | Simplify to `motor.Tick()` |
| `User/App/Src/CmdTask.cpp` | Use `SetXxxTarget()` and getters |
| `User/App/Src/user_main.cpp` | Use getters for Log::Print |
| `User/App/Inc/MotorTask.h` | Remove `extern float targetSpeed/targetCurrent/targetPosition` |

### 6. Implementation Order

1. Delete RATCHET/INERTIA code -> compile
2. Add `PidController` wrapper, switch FOC internal usage -> compile
3. Move FOC members to private, add getters/setters -> compile
4. Internalize state machine into `FOC::Tick()`, simplify MotorTask.cpp -> compile
5. Refactor CmdTask.cpp / user_main.cpp to use interfaces -> compile
