# FOC OOP Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor FOC core class and App layer for proper OOP encapsulation with zero-overhead guarantee.

**Architecture:** Incremental refactor — delete unused modes first, then wrap PID, tighten FOC encapsulation, internalize state machine, and update callers. Each task compiles independently.

**Tech Stack:** C++17 (ARMCLANG), STM32 HAL, FreeRTOS, EIDE build system

**Build command:** In VSCode, use EIDE command palette `EIDE: Build` or toolbar Build button. Alternatively:
```
"c:\Users\23763\.vscode\extensions\cl.eide-3.26.5\res\tools\win32\unify_builder" --config "build\FOCDriver_V2_F405RGT6\builder.params"
```

**Spec:** `docs/superpowers/specs/2026-03-28-foc-oop-refactor-design.md`

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `User/Lib/Inc/Foc.h` | Modify | Add PidController wrapper; reclassify members private; add getters/setters/target API |
| `User/Lib/Src/Foc.cpp` | Modify | Delete ratchet/inertia; implement Tick(), SetXxxTarget(); update MotorControlTask() |
| `User/App/Src/MotorTask.cpp` | Modify | Simplify to motor.Tick() |
| `User/App/Inc/MotorTask.h` | Modify | Remove stale extern declarations |
| `User/App/Src/CmdTask.cpp` | Modify | Use SetXxxTarget() and getters |
| `User/App/Src/user_main.cpp` | Modify | Use getters for Log::Print |

---

### Task 1: Delete RATCHET and INERTIA modes

**Files:**
- Modify: `User/Lib/Inc/Foc.h`
- Modify: `User/Lib/Src/Foc.cpp`

- [ ] **Step 1: Remove RATCHET/INERTIA from ControlMode enum in Foc.h**

In `User/Lib/Inc/Foc.h`, change the `ControlMode` enum from:

```cpp
enum class ControlMode : uint8_t
{
  NONE = 0,
  TORQUE = 1,
  VELOCITY = 2,
  POSITION = 3,
  MIT = 4,
  RATCHET = 5,
  INERTIA = 6,
};
```

To:

```cpp
enum class ControlMode : uint8_t
{
  NONE = 0,
  TORQUE = 1,
  VELOCITY = 2,
  POSITION = 3,
  MIT = 4,
};
```

- [ ] **Step 2: Remove ratchet and inertia member variables from FOC class in Foc.h**

Delete these lines from the `public:` section of class FOC:

```cpp
  float ratchetSpacing = (30.0f * _PI / 180.0f);
  float ratchetKp      = 0.5f;
  float ratchetKd      = 0.2f;

  float inertiaDecay             = 0.9999f;
  float inertiaMaxCurrent        = 0.3f;
  float inertiaKp                = 0.01f;
  float inertiaVelocityThreshold = 0.75f;
```

- [ ] **Step 3: Remove RATCHET/INERTIA case branches from MotorControlTask() in Foc.cpp**

In `User/Lib/Src/Foc.cpp`, in `FOC::MotorControlTask()`, delete the entire `case ControlMode::RATCHET:` block (lines ~492-503) and `case ControlMode::INERTIA:` block (lines ~505-507).

- [ ] **Step 4: Remove inertia loop logic from MotorControlTask() in Foc.cpp**

In `FOC::MotorControlTask()`, delete the entire `else if (controlMode == ControlMode::INERTIA)` block (lines ~538-568).

- [ ] **Step 5: Update default startup mode in MotorTask.cpp**

In `User/App/Src/MotorTask.cpp`, change the `MotorMode::NONE` case from:

```cpp
  case MotorMode::NONE:
    motor.motorMode = MotorMode::RUNNING;
    motor.targetSpeed = 0.0f;
    motor.velocitySetpoint = 0.0f;
    motor.controlMode = ControlMode::INERTIA;
    break;
```

To:

```cpp
  case MotorMode::NONE:
    motor.motorMode = MotorMode::RUNNING;
    motor.targetSpeed = 0.0f;
    motor.velocitySetpoint = 0.0f;
    motor.controlMode = ControlMode::TORQUE;
    motor.targetCurrent = 0.0f;
    break;
```

- [ ] **Step 6: Build and verify**

Build the project with EIDE. Expected: compiles with zero errors. No warnings related to removed code.

- [ ] **Step 7: Commit**

```bash
git add User/Lib/Inc/Foc.h User/Lib/Src/Foc.cpp User/App/Src/MotorTask.cpp
git commit -m "删除 RATCHET/INERTIA 模式，只保留力矩/速度/位置/MIT控制"
```

---

### Task 2: Add PidController C++ wrapper

**Files:**
- Modify: `User/Lib/Inc/Foc.h`
- Modify: `User/Lib/Src/Foc.cpp`

- [ ] **Step 1: Add PidController class definition in Foc.h**

In `User/Lib/Inc/Foc.h`, after the `#include "LowPassFilter.h"` line and before the `constrain` template, add:

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

    float* KpPtr() { return &pid_.Kp; }
    float* KiPtr() { return &pid_.Ki; }
    float* KdPtr() { return &pid_.Kd; }

    float Integral() const { return pid_.integral; }
    void ClearIntegral() { pid_.integral = 0.0f; }
};
```

- [ ] **Step 2: Replace PID member types in FOC class**

In `User/Lib/Inc/Foc.h`, replace the three PID member declarations. Change:

```cpp
  PIDControllerTypedef speedPidController;
  PIDControllerTypedef positionPidController;
  PIDControllerTypedef currentPidController;
```

To (keep in `public:` for now — will move to private in Task 3):

```cpp
  PidController speedPid;
  PidController positionPid;
  PidController currentPid;
```

- [ ] **Step 3: Update FOC::Init() to use PidController**

In `User/Lib/Src/Foc.cpp`, replace the three `PID_Init(...)` calls in `FOC::Init()`. Change:

```cpp
  PID_Init(&positionPidController,
          DEFAULT_POSITION_KP,
          DEFAULT_POSITION_KI,
          DEFAULT_POSITION_KD,
          0.001f,
          POSITION_PID_MAX_OUT * 0.75f, POSITION_PID_MAX_OUT, 0.0f,
          PID_MODE_POSITION);

  PID_Init(&speedPidController,
          DEFAULT_SPEED_KP,
          DEFAULT_SPEED_KI,
          DEFAULT_SPEED_KD,
          0.001f,
          SPEED_PID_MAX_OUT, SPEED_PID_MAX_OUT * 0.8f, 0.01f,
          PID_MODE_POSITION);

  PID_Init(&currentPidController,
          DEFAULT_CURRENT_KP,
          DEFAULT_CURRENT_KI,
          DEFAULT_CURRENT_KD,
          0.00005f,
          CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, 0.001f,
          PID_MODE_POSITION);
```

To:

```cpp
  positionPid.Init(
      DEFAULT_POSITION_KP, DEFAULT_POSITION_KI, DEFAULT_POSITION_KD,
      0.001f, POSITION_PID_MAX_OUT * 0.75f, POSITION_PID_MAX_OUT, 0.0f,
      PID_MODE_POSITION);

  speedPid.Init(
      DEFAULT_SPEED_KP, DEFAULT_SPEED_KI, DEFAULT_SPEED_KD,
      0.001f, SPEED_PID_MAX_OUT, SPEED_PID_MAX_OUT * 0.8f, 0.01f,
      PID_MODE_POSITION);

  currentPid.Init(
      DEFAULT_CURRENT_KP, DEFAULT_CURRENT_KI, DEFAULT_CURRENT_KD,
      0.00005f, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, 0.001f,
      PID_MODE_POSITION);
```

- [ ] **Step 4: Update all PIDCompute/PID_SetOutputLimits calls in Foc.cpp**

In `FOC::MotorControlTask()`, replace every occurrence. The full list:

| Old | New |
|-----|-----|
| `PID_SetOutputLimits(&currentPidController, ...)` | `currentPid.SetOutputLimits(...)` |
| `PID_SetOutputLimits(&speedPidController, ...)` | `speedPid.SetOutputLimits(...)` |
| `PID_SetOutputLimits(&positionPidController, ...)` | `positionPid.SetOutputLimits(...)` |
| `PIDCompute(&currentPidController, ...)` | `currentPid.Compute(...)` |
| `PIDCompute(&speedPidController, ...)` | `speedPid.Compute(...)` |
| `PIDCompute(&positionPidController, ...)` | `positionPid.Compute(...)` |

Also in `FOC::PositionControl()` and `FOC::SpeedControl()`:

| Old | New |
|-----|-----|
| `PIDCompute(&positionPidController, ...)` | `positionPid.Compute(...)` |
| `PIDCompute(&speedPidController, ...)` | `speedPid.Compute(...)` |

- [ ] **Step 5: Update EmergencyStop() in Foc.cpp**

Change:

```cpp
  speedPidController.integral = 0.0f;
  positionPidController.integral = 0.0f;
  currentPidController.integral = 0.0f;
```

To:

```cpp
  speedPid.ClearIntegral();
  positionPid.ClearIntegral();
  currentPid.ClearIntegral();
```

- [ ] **Step 6: Update SetPIDParameters() in Foc.cpp**

Replace the entire `FOC::SetPIDParameters()` method body:

```cpp
void FOC::SetPIDParameters(float kp_speed, float ki_speed, float kd_speed,
                          float kp_pos, float ki_pos, float kd_pos,
                          float kp_current, float ki_current, float kd_current)
{
  *speedPid.KpPtr() = kp_speed;
  *speedPid.KiPtr() = ki_speed;
  *speedPid.KdPtr() = kd_speed;

  *positionPid.KpPtr() = kp_pos;
  *positionPid.KiPtr() = ki_pos;
  *positionPid.KdPtr() = kd_pos;

  *currentPid.KpPtr() = kp_current;
  *currentPid.KiPtr() = ki_current;
  *currentPid.KdPtr() = kd_current;
}
```

- [ ] **Step 7: Build and verify**

Build the project with EIDE. Expected: compiles with zero errors.

- [ ] **Step 8: Commit**

```bash
git add User/Lib/Inc/Foc.h User/Lib/Src/Foc.cpp
git commit -m "添加 PidController C++ wrapper，替换原始 PID struct 调用"
```

---

### Task 3: Tighten FOC class encapsulation

**Files:**
- Modify: `User/Lib/Inc/Foc.h`

This is a header-only change — move members to private and add inline accessors.

- [ ] **Step 1: Reorganize the FOC class members and add getters/setters**

Replace the entire `class FOC { ... };` in `User/Lib/Inc/Foc.h` with the following. This keeps all existing methods but reorganizes members into private with inline accessors:

```cpp
class FOC
{
private:
  // Internal electrical state
  float uAlpha_ = 0.0f, uBeta_ = 0.0f;
  float currentAOffset_ = 0.0f, currentBOffset_ = 0.0f, currentCOffset_ = 0.0f;
  float Vbus_ = 0.0f;

  // Measured currents
  float Ia_ = 0.0f, Ib_ = 0.0f, Ic_ = 0.0f;
  float Iq_ = 0.0f, Id_ = 0.0f;

  // Phase voltages (internal)
  float voltageA_ = 0.0f, voltageB_ = 0.0f, voltageC_ = 0.0f;
  float voltageA_Duty_ = 0.0f, voltageB_Duty_ = 0.0f, voltageC_Duty_ = 0.0f;

  // Sensor state
  float angleSingleTurn_ = 0.0f;
  float motorAngle_ = 0.0f;
  float velocity_ = 0.0f;

  // Control targets
  float targetPosition_ = 0.0f;
  float targetSpeed_ = 0.0f;
  float targetCurrent_ = 0.0f;
  float targetTorque_ = 0.0f;

  // MIT params
  float mitKp_ = 0.0f;
  float mitKd_ = 0.0f;

  // Internal setpoints
  float iqSetpoint_ = 0.0f;
  float idSetpoint_ = 0.0f;
  float positionSetpoint_ = 0.0f;
  float velocitySetpoint_ = 0.0f;

  // State machine
  MotorMode motorMode_ = MotorMode::INIT;
  ControlMode controlMode_ = ControlMode::TORQUE;

  // Hardware
  TIM_HandleTypeDef* pwmTimer_ = nullptr;

  // Filters
  LPS speedFilter_{1000.0f, 100.0f};
  LPS IqFilter_{20000.0f, 40.0f};
  LPS IdFilter_{20000.0f, 40.0f};
  LPS IaFilter_{20000.0f, 1000.0f};
  LPS IbFilter_{20000.0f, 1000.0f};
  LPS IcFilter_{20000.0f, 1000.0f};

  // PID controllers
  PidController speedPid_;
  PidController positionPid_;
  PidController currentPid_;

  // Loop counter for velocity decimation
  uint8_t loopCount_ = 0;

#ifdef YAW_MOTOR
  float zeroElectricAngle_ = 1.5f;
#else
  float zeroElectricAngle_ = 0.0f;
#endif

  // Private methods
  void SetPwm();
  void SvpwmSector();
  void ClarkeParkTransform(float electricalAngle);
  float NormalizeAngle(float angle);
  float GetElectricAngle(float shaftAngle);
  void SetPhaseVoltage(float uq, float ud, float electricalAngle);
  float GetVelocity(float angle);
  float LowPassFilterUpdate(LowPassFilterTypedef *filter, float input);
  void UpdateCurrent();
  void UpdateCurrentOffsets();
  void UpdateMotorAngle();
  void UpdateVelocity();
  void MotorControlTask();

public:
  FOC();
  ~FOC();

  // Lifecycle
  bool Init(TIM_HandleTypeDef* timer = &htim1);
  void Tick();  // Main entry: call from ISR
  void CalibrateZeroElectricAngle();
  void EmergencyStop();

  // Target-setting API (sets mode implicitly)
  void SetTorqueTarget(float current);
  void SetVelocityTarget(float speed);
  void SetPositionTarget(float position);
  void SetMitTarget(float pos, float vel, float torque, float kp, float kd);
  void Stop();

  // Read-only getters
  float Ia() const { return Ia_; }
  float Ib() const { return Ib_; }
  float Ic() const { return Ic_; }
  float Iq() const { return Iq_; }
  float Id() const { return Id_; }
  float MotorAngle() const { return motorAngle_; }
  float AngleSingleTurn() const { return angleSingleTurn_; }
  float Velocity() const { return velocity_; }
  float TargetPosition() const { return targetPosition_; }
  float TargetSpeed() const { return targetSpeed_; }
  float TargetCurrent() const { return targetCurrent_; }
  float TargetTorque() const { return targetTorque_; }
  ControlMode GetControlMode() const { return controlMode_; }
  MotorMode GetMotorMode() const { return motorMode_; }

  // VOFA pointer access (for online tuning)
  float* TargetPositionPtr() { return &targetPosition_; }
  float* TargetSpeedPtr() { return &targetSpeed_; }
  float* TargetCurrentPtr() { return &targetCurrent_; }
  float* TargetTorquePtr() { return &targetTorque_; }

  // PID access (for VOFA tuning)
  PidController& SpeedPid() { return speedPid_; }
  PidController& PositionPid() { return positionPid_; }
  PidController& CurrentPid() { return currentPid_; }

  // Legacy API (kept for SetPIDParameters compatibility)
  void SetPIDParameters(float kp_speed, float ki_speed, float kd_speed,
                        float kp_pos, float ki_pos, float kd_pos,
                        float kp_current, float ki_current, float kd_current);
  void SetMotorParameters(int polePairs, float voltageLimit, float powerSupply = VOLTAGE_POWER_SUPPLY);

  // Kept for external use if needed
  void SpeedControl(float targetSpeed);
  void TorqueControl(float targetTorque);
  void PositionControl(float targetPosition);
  void GetMechanicalAngle();
};
```

- [ ] **Step 2: Update all member references in Foc.cpp**

In `User/Lib/Src/Foc.cpp`, rename every member variable reference to use the trailing-underscore convention. Complete mapping:

| Old name | New name |
|----------|----------|
| `uAlpha` | `uAlpha_` |
| `uBeta` | `uBeta_` |
| `currentAOffset` | `currentAOffset_` |
| `currentBOffset` | `currentBOffset_` |
| `currentCOffset` | `currentCOffset_` |
| `Vbus` | `Vbus_` |
| `Ia` | `Ia_` |
| `Ib` | `Ib_` |
| `Ic` | `Ic_` |
| `Iq` | `Iq_` |
| `Id` | `Id_` |
| `voltageA` | `voltageA_` |
| `voltageB` | `voltageB_` |
| `voltageC` | `voltageC_` |
| `voltageA_Duty` | `voltageA_Duty_` |
| `voltageB_Duty` | `voltageB_Duty_` |
| `voltageC_Duty` | `voltageC_Duty_` |
| `angleSingleTurn` | `angleSingleTurn_` |
| `motorAngle` | `motorAngle_` |
| `velocity` | `velocity_` |
| `targetPosition` | `targetPosition_` |
| `targetSpeed` | `targetSpeed_` |
| `targetCurrent` | `targetCurrent_` |
| `targetTorque` | `targetTorque_` |
| `mitKp` | `mitKp_` |
| `mitKd` | `mitKd_` |
| `iqSetpoint` | `iqSetpoint_` |
| `idSetpoint` | `idSetpoint_` |
| `positionSetpoint` | `positionSetpoint_` |
| `velocitySetpoint` | `velocitySetpoint_` |
| `motorMode` | `motorMode_` |
| `controlMode` | `controlMode_` |
| `pwmTimer` | `pwmTimer_` |
| `speedFiliter` | `speedFilter_` |
| `IqFilter` | `IqFilter_` |
| `IdFilter` | `IdFilter_` |
| `IaFilter` | `IaFilter_` |
| `IbFilter` | `IbFilter_` |
| `IcFilter` | `IcFilter_` |
| `zeroElectricAngle` | `zeroElectricAngle_` |
| `speedPid` | `speedPid_` |
| `positionPid` | `positionPid_` |
| `currentPid` | `currentPid_` |

Also remove the `static uint8_t loopCount` local variable from `MotorControlTask()` — it's now the member `loopCount_`.

- [ ] **Step 3: Build and verify**

Build the project with EIDE. Expected: compile errors in MotorTask.cpp, CmdTask.cpp, user_main.cpp because they still access old public members. This is expected — we fix them in Tasks 4 and 5.

**Workaround for interim compilation:** Temporarily leave this task's commit for after Tasks 4-5 are done, OR do Tasks 3+4+5 as one atomic change. Recommended: do them together — the header change and caller updates are tightly coupled.

- [ ] **Step 4: Commit (together with Task 4 and Task 5)**

Defer commit until all callers are updated.

---

### Task 4: Internalize state machine into FOC::Tick() and add target-setting API

**Files:**
- Modify: `User/Lib/Src/Foc.cpp`
- Modify: `User/App/Src/MotorTask.cpp`
- Modify: `User/App/Inc/MotorTask.h`

- [ ] **Step 1: Implement FOC::Tick() in Foc.cpp**

Add the following method to `User/Lib/Src/Foc.cpp`:

```cpp
void FOC::Tick()
{
  UpdateCurrent();
  UpdateMotorAngle();
  loopCount_++;
  if (loopCount_ >= 20)
  {
    loopCount_ = 0;
    UpdateVelocity();
  }

  switch (motorMode_)
  {
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

  default:
    break;
  }
}
```

- [ ] **Step 2: Implement target-setting methods in Foc.cpp**

Add the following methods to `User/Lib/Src/Foc.cpp`:

```cpp
void FOC::SetTorqueTarget(float current)
{
  targetCurrent_ = current;
  controlMode_ = ControlMode::TORQUE;
}

void FOC::SetVelocityTarget(float speed)
{
  targetSpeed_ = speed;
  controlMode_ = ControlMode::VELOCITY;
}

void FOC::SetPositionTarget(float position)
{
  targetPosition_ = position;
  controlMode_ = ControlMode::POSITION;
}

void FOC::SetMitTarget(float pos, float vel, float torque, float kp, float kd)
{
  targetPosition_ = pos;
  targetSpeed_ = vel;
  targetTorque_ = torque;
  mitKp_ = kp;
  mitKd_ = kd;
  controlMode_ = ControlMode::MIT;
}

void FOC::Stop()
{
  controlMode_ = ControlMode::NONE;
  iqSetpoint_ = 0.0f;
  idSetpoint_ = 0.0f;
  velocitySetpoint_ = 0.0f;
  positionSetpoint_ = 0.0f;
  SetPhaseVoltage(0.0f, 0.0f, 0.0f);
}
```

- [ ] **Step 3: Simplify MotorTask.cpp**

Replace the entire content of `User/App/Src/MotorTask.cpp` with:

```cpp
#include "MotorTask.h"
#include "Foc.h"

FOC motor;

void MotorTask()
{
  motor.Tick();
}
```

- [ ] **Step 4: Clean up MotorTask.h**

Replace the content of `User/App/Inc/MotorTask.h` with:

```cpp
#ifndef __MOTOR_TASK_H__
#define __MOTOR_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "common_inc.h"
#include "Foc.h"

extern FOC motor;

extern "C"
{
void MotorTask();
}
#endif

#endif // !__MOTOR_TASK_H__
```

This removes the stale `extern float targetSpeed/targetCurrent/targetPosition` declarations.

---

### Task 5: Refactor CmdTask.cpp and user_main.cpp to use interfaces

**Files:**
- Modify: `User/App/Src/CmdTask.cpp`
- Modify: `User/App/Src/user_main.cpp`

- [ ] **Step 1: Update AnalyzeCmd() in CmdTask.cpp**

Replace the `AnalyzeCmd()` function body in `User/App/Src/CmdTask.cpp`:

```cpp
void AnalyzeCmd(uint32_t ctrlId)
{
  switch (ctrlId)
  {
  case MotorCanExtId::TORQUE_CTRL:
  {
    float torqueRx;
    memcpy(&torqueRx, &canRxBuffer[0], sizeof(float));
    motor.SetTorqueTarget(torqueRx);
    break;
  }
  case MotorCanExtId::VELOCITY_CTRL:
  {
    float velocityRx;
    memcpy(&velocityRx, &canRxBuffer[0], sizeof(float));
    motor.SetVelocityTarget(velocityRx);
    break;
  }
  case MotorCanExtId::POSITION_CTRL:
  {
    float positionRx;
    memcpy(&positionRx, &canRxBuffer[0], sizeof(float));
    motor.SetPositionTarget(positionRx);
    break;
  }
  case MotorCanExtId::MIT_CTRL:
    GetMitCmd(canRxBuffer, &mitCmd_Rx);
    motor.SetMitTarget(mitCmd_Rx.position, mitCmd_Rx.velocity,
                       mitCmd_Rx.torque, mitCmd_Rx.kp, mitCmd_Rx.kd);
    break;
  default:
    break;
  }
}
```

- [ ] **Step 2: Update SendStatusCmd() in CmdTask.cpp**

Replace the `SendStatusCmd()` function body:

```cpp
void SendStatusCmd()
{
  float position = motor.AngleSingleTurn();
  float velocity = motor.Velocity();
  float torque   = motor.Iq() * TORQUE_CONST;

  uint16_t velInt    = FloatToUint(velocity, -200, 200, 16);
  uint16_t torqueInt = FloatToUint(torque, -TORQUE_LIMIT * 2, TORQUE_LIMIT * 2, 16);

  uint8_t statusCmd[8];

  memcpy(&statusCmd[0], &position, sizeof(float));
  statusCmd[4] = velInt >> 8;
  statusCmd[5] = velInt & 0xFF;
  statusCmd[6] = torqueInt >> 8;
  statusCmd[7] = torqueInt & 0xFF;

  can1.SendStdData(MOTOR_ID + MotorCanExtId::STATUS_FEEDBACK, statusCmd, 8);
}
```

- [ ] **Step 3: Update user_main.cpp**

In `User/App/Src/user_main.cpp`, change the Log::Print line in the while loop:

From:
```cpp
Log::Print("%f, %f, %f, %f\n", motor.motorAngle, motor.velocity, motor.targetCurrent, motor.Iq);
```

To:
```cpp
Log::Print("%f, %f, %f, %f\n", motor.MotorAngle(), motor.Velocity(), motor.TargetCurrent(), motor.Iq());
```

- [ ] **Step 4: Remove unnecessary includes in CmdTask.cpp**

In `User/App/Src/CmdTask.cpp`, the `extern "C" { #include "VOFA.h" }` block can be removed if VOFA is not used directly in this file. Check if it's used — if not, remove it.

- [ ] **Step 5: Build and verify**

Build the project with EIDE. Expected: compiles with zero errors and zero warnings related to changed code.

- [ ] **Step 6: Commit all of Tasks 3, 4, 5 together**

```bash
git add User/Lib/Inc/Foc.h User/Lib/Src/Foc.cpp User/App/Src/MotorTask.cpp User/App/Inc/MotorTask.h User/App/Src/CmdTask.cpp User/App/Src/user_main.cpp
git commit -m "收紧 FOC 封装：成员 private 化、添加 getter/setter、状态机内化、统一目标设置 API"
```
