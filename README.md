# RoBug
RoBug is a small quadruped robot that combines simplicity, affordability and advanced
robotics features under one roof. RoBug was designed to be both, a beginner friendly
project for everybody who wants to dive into robotics without spending a fortune for
parts and tools, but also an open, modular and feature-rich platform for advanced
robotics experiments. No matter if you want to navigate RoBug through your room with
the RoBug Remote Control App running on your mobile phone or if you want to experiment
with autonomous navigation or obstacle detection, this project on hackster.io is for you:
[robug - tiny robot, big attitude](https://www.hackster.io/robots4all/robug-tiny-robot-big-attitude-ec32c7)

<img width="800" height="600" alt="RoBug_and_his_double" src="https://github.com/user-attachments/assets/dab777c2-4b4e-4f69-9bd3-a20df38d0b9e" />

# RoBug Firmware Documentation

## Overview

RoBug firmware is a MicroPython-based control system for an affordable, palm-sized quadruped robot. The firmware implements inverse kinematics, gait generation, motion control, and Bluetooth-based remote operation. This document provides a comprehensive guide to the firmware architecture and key components.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Core Components](#core-components)
3. [Initialization and Main Loop](#initialization-and-main-loop)
4. [Gait System](#gait-system)
5. [Inverse Kinematics](#inverse-kinematics)
6. [Hardware Control](#hardware-control)
7. [Communication](#communication)
8. [Motion Controller](#motion-controller)
9. [Application Framework](#application-framework)
10. [Configuration](#configuration)

---

## Architecture Overview

The firmware follows a modular, asynchronous architecture using MicroPython's `asyncio` library. Multiple concurrent tasks handle different aspects of robot control:

```
┌─────────────────────────────────────────────────────────┐
│                     main.py                             │
│  (Entry point - initializes and runs all tasks)         │
└──────────────┬──────────────────────────────────────────┘
               │
       ┌───────┼────────┬──────────┬─────────────┐
       │       │        │          │             │
    ┌──▼──┐ ┌─▼──┐ ┌───▼─┐ ┌────▼──┐ ┌─────▼──┐
    │ BLE │ │IK  │ │Gait │ │Motion │ │Sensors │
    │     │ │    │ │     │ │Ctrl   │ │        │
    └─────┘ └────┘ └─────┘ └───────┘ └────────┘
       │       │        │          │             │
       └───────┴────────┴──────────┴─────────────┘
               │
         ┌─────▼──────┐
         │Robot Model │
         │  (Legs)    │
         └────────────┘
```

### Key Design Principles

- **Modular**: Each component (gait, IK, control) is independent
- **Asynchronous**: Non-blocking task scheduling for real-time operation
- **Hardware Abstraction**: Constants define hardware specifics
- **Calibration**: JSON-based servo calibration for flexible configuration

---

## Core Components

### 1. **robug_robot.py** - Robot Model

The `robug` class represents the complete robot and manages all four legs.

**Key Methods:**
- `__init__()`: Initializes robot with leg objects, sensors, and LEDs
- `create_robug()`: Loads calibration data and instantiates leg objects
- `instant_update()`: Updates joint angles and sets servos immediately
- `calculate_joint_angles()`: Computes angles for all legs
- `set_positions()`: Updates leg foot positions (Cartesian coordinates)
- `set_direction()`: Controls forward/rotation direction for walking

**Hardware Interfaces:**
- **Sensors**: Touch sensors (top/bottom), distance sensor (VL53L0X)
- **Actuators**: 8 servo motors (2 per leg: femur + tibia)
- **LEDs**: Red and green status LEDs with PWM control

```python
r = robug()
r.instant_update()  # Synchronize all leg positions
r.set_direction(1, 'x')  # Forward direction
```

---

### 2. **robug_leg.py** - Individual Leg Control

The `rbleg` class manages a single leg with its two servo joints and gait generator.

**Structure:**
```
rbleg (one per quadrant)
├── Gait Generator (rbgait)
├── Inverse Kinematics Solver (rbik)
├── Joint Controller (rbjoint) - 2 servos
└── Foot Position (v3 vector)
```

**Key Attributes:**
- `gait`: Gait generator for this leg
- `foot_pos`: Current foot position in 3D space
- `joints`: Servo controller

---

### 3. **robug_gait.py** - Gait Generator

The `rbgait` class generates smooth leg trajectories following a trotting gait pattern.

**Gait Phases:**
```
Phase 1: Support Start (cycle start, foot on ground)
Phase 2: Support Phase (locomotion, foot on ground - majority of cycle)
Phase 3: Support End (foot leaving ground)
Phase 4: Swing Phase (foot in air, returning to start)
```

**Key Concepts:**

- **Loop Counter (`i`)**: Tracks position in gait cycle (0 to `substeps`)
- **Stride Length**: Distance leg travels per cycle (controlled by `_GAIT_HALF_STRIDE`)
- **Swing Amplitude**: Height of leg lift during swing phase
- **Support/Swing Ratio**: Adjustable timing between support and swing phases

**Foot Trajectory Calculation:**

For **support phase** (leg on ground):
- X: Linear trajectory from max to min position
- Z: Stays at `_GAIT_HEIGHT` (ground contact)

For **swing phase** (leg in air):
- X: Linear return trajectory from min to max
- Z: Sinusoidal arc (apex height = `_GAIT_HEIGHT + _GAIT_SWING_AMPL`)

**Push Mechanism:**
During swing phase of opposite leg pair, support legs push downward with additional force (`_GAIT_PUSH_STRENGTH`) to carry load and stabilize the body.

```python
gait = rbgait(leg_id=0)  # Front left leg
gait.loop_inc()  # Advance gait by one tick
xyz = gait.get_xyz()  # Get current foot position
```

---

### 4. **robug_ik.py** - Inverse Kinematics Solver

Solves inverse kinematics for the 2-DOF leg (femur + tibia).

**Given:**
- Target foot position (x, z)
- Leg link lengths: femur (`_L_FEMUR`), tibia (`_L_TIBIA`)
- Hip offset from body center (`_DIST_PIVOT_TO_HIP`)

**Computes:**
- Femur joint angle
- Tibia joint angle

**Algorithm:** Analytical 2-link IK with geometric solutions.

---

### 5. **robug_joints.py** - Joint/Servo Control

The `rbjoint` class manages individual servo motors.

**Responsibilities:**
- Servo angle-to-PWM conversion
- Applying calibration offsets
- Applying gain corrections
- Enforcing servo limits

**Servo Mapping:**
```
Leg 0 (Front-Left):   Servos 0-1 (femur, tibia)
Leg 1 (Rear-Left):    Servos 2-3 (femur, tibia)
Leg 2 (Front-Right):  Servos 4-5 (femur, tibia)
Leg 3 (Rear-Right):   Servos 6-7 (femur, tibia)
```

---

### 6. **robug_constants.py** - Configuration

Centralized hardware and behavior parameters.

**Hardware Configuration (by version):**
- Pin mappings for servo motors, sensors, LEDs
- I2C bus configuration
- ADC pin for battery voltage

**Gait Parameters:**
- `_GAIT_LOOP_TIME`: Update interval (8ms - fast, 11ms - smooth)
- `_GAIT_SWING_TICKS`: Duration of swing phase
- `_GAIT_SUPPORT_TICKS`: Duration of support phase
- `_GAIT_HEIGHT`: Foot height relative to ground
- `_GAIT_SWING_AMPL`: Peak height during swing

**Physical Parameters:**
- `_L_FEMUR`: Femur length (47.1mm)
- `_L_TIBIA`: Tibia length (111.6mm)
- `_SERVO_K`: Servo angle constant

**Example: Slow/Smooth Gait**
```python
_GAIT_LOOP_TIME = 11      # Slower updates
_GAIT_SWING_TICKS = 14    # Longer swing
_GAIT_SUPPORT_TICKS = 64  # Longer support
```

---

## Initialization and Main Loop

### Startup Sequence

**main.py - Initialization Phase:**

```python
# 1. Initialize BLE communication
ble = rbble()

# 2. Create robot instance and load calibration
r = robug()
r.set_loop_counter_resume()  # Diagonal leg phase offset
r.instant_update()  # Apply initial positions

# 3. Setup battery monitoring
adc = ADC(c._PIN_ADC)

# 4. Create motion controller with message queues
MsgQueue = deque([], 4)
RplyQueue = deque([], 4)
rc = rbctrl(MsgQueue, RplyQueue)
m = rbmocon(r, MsgQueue, RplyQueue)
```

### Concurrent Tasks

The main loop runs multiple asynchronous tasks:

```python
async def main():
    # Communication
    task_ble = asyncio.create_task(ble.msg_handler())
    
    # Sensors
    task_dist = asyncio.create_task(serve_sensor_data())
    task_adc = asyncio.create_task(get_battery_voltage())
    
    # Actuators
    task_led = asyncio.create_task(pulse_leds())
    task_mc = asyncio.create_task(m.run())
    
    # Supervisor (handles remote control commands)
    task_rc = asyncio.create_task(fpv_rc(ble))
    
    await task_rc  # Main loop runs until RC task completes
```

**Task Responsibilities:**
- `msg_handler()`: Process incoming BLE commands
- `serve_sensor_data()`: Poll distance sensor at 4Hz
- `get_battery_voltage()`: Monitor battery at 2Hz
- `pulse_leds()`: Animate status LEDs
- `run()`: Motion controller main loop
- `fpv_rc()`: Remote control state machine

---

## Gait System

### Trotting Gait Pattern

RoBug uses a trotting pattern where diagonal legs move together:

```
Timeline:
Cycle Start → Support Phase → Swing Phase → Cycle End

Legs:
FL (Front-Left)  ↓                ↓ (support)  ↑ (swing)
RL (Rear-Left)   ↑ (swing)        ↓ (support)
FR (Front-Right) ↑ (swing)        ↓ (support)
RR (Rear-Right)  ↓ (support)      ↑ (swing)

Diagonal pairs move together:
- FL + RR: Same phase (diagonal pair 1)
- RL + FR: Same phase (diagonal pair 2)
```

**Phase Offset Configuration:**
```python
_GAIT_PHASE_OFFSET = (SUPPORT_TICKS + SWING_TICKS) / 2
_GAIT_LEG_PHASE_OFFSET = [0, offset, offset, 0]
```

### Trajectory Generation

**During Support Phase:**
```
Foot moves linearly from max X to min X (relative to body)
Height remains at _GAIT_HEIGHT (on ground)
Leg pushes against ground, body moves forward
```

**During Swing Phase:**
```
Foot moves linearly from min X back to max X
Height follows sinusoidal arc (lifted off ground)
Leg returns to starting position for next cycle
```

### Tuning Parameters for Different Behaviors

**Fast Gait (8ms loop):**
```python
_GAIT_LOOP_TIME = 8
_GAIT_SWING_TICKS = 12
_GAIT_SUPPORT_TICKS = 40
_GAIT_SWING_AMPL = 13
_GAIT_HALF_STRIDE = 37
```

**Smooth Gait (11ms loop):**
```python
_GAIT_LOOP_TIME = 11
_GAIT_SWING_TICKS = 14
_GAIT_SUPPORT_TICKS = 64
_GAIT_HEIGHT = -110
_GAIT_SWING_AMPL = -15
_GAIT_HALF_STRIDE = 35
```

---

## Inverse Kinematics

### Problem Statement

Given a target foot position (x, z), find joint angles (θ₁, θ₂) such that the foot reaches that position.

**Constraints:**
- Femur and tibia are rigid links
- Movement is in the x-z plane (sagittal plane)
- Only 2 degrees of freedom per leg

### Solution Approach

Analytical IK using law of cosines:

```
d = distance from hip to target
θ₂ = acos((d² - L₁² - L₂²) / (2·L₁·L₂))  [Knee angle]
θ₁ = atan2(z, x) - atan2(L₂·sin(θ₂), L₁ + L₂·cos(θ₂))  [Hip angle]
```

### Implementation

```python
solver = rbik()
angles = solver.solve_ik(foot_position, leg_id)
# Returns: (femur_angle, tibia_angle)
```

---

## Hardware Control

### Servo Control System

**Servo PWM Parameters:**
- Frequency: 250 Hz (standard RC servo)
- Neutral pulse: 1500 µs (90°)
- Min pulse: 400 µs (max angle -90°)
- Max pulse: 2600 µs (max angle +90°)

**Angle to PWM Conversion:**
```
PWM_ticks = Neutral + (angle_rad * SERVO_K) + Calibration_offset
PWM_ticks = PWM_ticks * Gain_correction
```

### Servo Calibration (robug_calibration.json)

Stored calibration data compensates for manufacturing variations:

```json
{
  "robug_calibration_data": {
    "servo_offs": [offset0, offset1, ..., offset7],
    "servo_gain": [gain0, gain1, ..., gain7]
  }
}
```

**Calibration Tool:** `robug_app_calibrator.py` - Interactive servo positioning and calibration.

### Sensor Integration

**Distance Sensor (VL53L0X):**
- I2C interface
- Measures distance for obstacle avoidance
- Polled at 4Hz from `serve_sensor_data()` task
- Median of 6 samples for noise filtering

**Touch Sensors:**
- Digital GPIO inputs
- Top sensor: Petting detection
- Bottom sensor: Collision/ground feedback

**Battery Monitor:**
- ADC input with voltage divider (47k/100k)
- Scaled to 3.3V logic level
- Monitored for low-battery warnings

### LED Control

**Status LEDs:**
- Green LED: Power/activity indicator
- Red LED: Error/warning indicator
- PWM control for brightness/pulsing effects

**Pulse Animation:**
```python
async def pulse_leds():
    while True:
        grnPct = 50 + sin(angle) * 50
        redPct = 15 + sin(angle + π) * 15
        r.set_brightness_grn(grnPct)
        r.set_brightness_red(redPct)
        angle += increment
```

---

## Communication

### Bluetooth Low Energy (BLE)

**Module:** `robug_ble.py`

**Features:**
- Wireless command reception
- Real-time sensor data transmission
- Low-power operation

**Command Types:**
- Movement: FWD, BWD, LEFT, RIGHT, STOP
- Posture: SIT_DOWN, STAND_UP
- Special: Calibration, LED control

**Data Transmission:**
- Distance sensor readings
- Battery voltage
- Robot state

### Message Queue Architecture

Motion control uses inter-task communication:

```
BLE Task → MsgQueue → Motion Controller Task
                  ← RplyQueue → Sensor Task
```

**Message Format:**
- Commands: Movement directives, parameters
- Replies: Status confirmations, sensor data

---

## Motion Controller

### Module: robug_mocon.py

Processes movement commands and orchestrates gait execution.

**Responsibilities:**
- Parse incoming movement commands
- Update gait parameters (speed, direction)
- Manage state transitions (idle → walking → stopped)
- Coordinate with robot model for leg updates
- Maintain timing loop

**Main Loop:**
```python
async def run(self):
    while True:
        cmd = get_message_from_queue()
        if cmd == 'FORWARD':
            self.start_forward_walk()
        elif cmd == 'STOP':
            self.stop_walk()
        
        self.robot.inc_loop_counters()  # Advance gait
        self.robot.calculate_joint_angles()
        self.robot.set_joints()
        
        await asyncio.sleep_ms(GAIT_LOOP_TIME)
```

### Movement Commands

**Walk Forward:**
```python
await rc.start_to_walk_fwd()
# Sets gait direction to +1 (x-axis forward)
```

**Walk Backward:**
```python
await rc.start_to_walk_bwd()
# Sets gait direction to -1 (x-axis backward)
```

**Turn:**
```python
await rc.turn(direction, angle_increments)
# direction: 1 (right), -1 (left)
# Modulates leg stride asymmetrically
```

**Posture Control:**
```python
await rc.sit_down()      # Lower body to ground
await rc.stand_up()      # Raise body to walking height
await rc.init_pose()     # Return to neutral stance
```

---

## Application Framework

### Application Template

The firmware supports multiple concurrent applications (apps) via `robug_app_*.py` modules.

**Template Structure:**
```python
class MyApp:
    def __init__(self, robot, msg_queue, reply_queue):
        self.robot = robot
        self.msg_queue = msg_queue
        self.reply_queue = reply_queue
    
    async def run(self):
        while True:
            # Application logic
            await asyncio.sleep_ms(100)
```

### Built-in Applications

1. **robug_app_fpv.py** - First-Person View camera streaming
2. **robug_app_explorer.py** - Autonomous exploration with obstacle avoidance
3. **robug_app_calibrator.py** - Interactive servo calibration tool
4. **robug_app_template.py** - Starter template for custom apps

### Example: Remote Control App (main.py)

```python
async def fpv_rc(ble):
    state = 'init'
    
    while True:
        cmd = ble.get_current_cmd()
        
        if state == 'init':
            await rc.init_pose()
            state = 'idle'
        
        elif state == 'idle':
            if cmd == 'FWD':
                await rc.start_to_walk_fwd()
                state = 'forward'
            elif cmd == 'SIT_DOWN':
                await rc.sit_down()
                state = 'sitting'
        
        elif state == 'forward':
            if cmd == 'STOP':
                await rc.stop_fwd()
                state = 'idle'
        
        # ... more state transitions ...
        
        await asyncio.sleep_ms(100)
```

---

## Configuration

### Local Configuration

**File:** `robug_local_config.py` (gitignored)

Override default constants:
```python
_HW = 'V100'  # Hardware version (prototype, V100)
```

**Template:** `robug_local_config.py.example`

### Hardware Versions

**Prototype:**
```python
_SERVO_MAP = [16, 17, 19, 18, 14, 15, 13, 12]
_PIN_TOUCH_TOP = 0
_PIN_TOUCH_BOT = 1
```

**V100:**
```python
_SERVO_MAP = [12, 11, 4, 3, 15, 14, 1, 0]
_PIN_TOUCH_TOP = 6
_PIN_TOUCH_BOT = 19
```

### Calibration Data

**File:** `robug_calibration.json`

Stores per-unit servo calibration:
```json
{
  "robug_calibration_data": {
    "servo_offs": [-20, 20, 35, 10, 15, 20, 20, 0],
    "servo_gain": [0.95, 1.00, 0.90, 1.03, 0.97, 0.93, 0.93, 0.97]
  }
}
```

Generated by calibration tool.

---

## Debugging and Testing

### Utility Modules

**robug_utils.py:**
- `v3` class: 3D vector operations for foot positions

**robug_led_test.py:**
- Test script for LED functionality

**robug_constants.py:**
- `#GAIT_LOOP_TIME = 11` - Uncomment for debug timing

### Common Issues and Solutions

**Servo Jitter:**
- Increase `_GAIT_LOOP_TIME` for slower updates
- Check servo calibration offsets

**Unstable Walking:**
- Adjust `_GAIT_HEIGHT` (lower = more ground contact)
- Increase `_GAIT_PUSH_STRENGTH` for body support
- Tune `_GAIT_SWING_AMPL` for clearance

**Asymmetric Gait:**
- Verify servo gains and offsets per leg
- Check `_LEG_DIR` direction corrections

**BLE Latency:**
- Reduce sensor polling rates if needed
- Increase `MsgQueue` size for buffering

---

## File Reference

| File | Purpose |
|------|---------|
| `main.py` | Entry point, task orchestration |
| `robug_robot.py` | Robot model, leg management |
| `robug_leg.py` | Single leg control |
| `robug_gait.py` | Trajectory generation |
| `robug_ik.py` | Inverse kinematics solver |
| `robug_joints.py` | Servo motor control |
| `robug_ctrl.py` | Command interpreter |
| `robug_mocon.py` | Motion controller |
| `robug_ble.py` | Bluetooth communication |
| `robug_constants.py` | Centralized configuration |
| `robug_calibration.py` | Calibration loader |
| `robug_com.py` | Serial communication utilities |
| `robug_utils.py` | Utility classes (v3, etc.) |
| `tof_sensor.py` | Distance sensor driver |
| `robug_calibration.json` | Calibration data |

---

## Advanced Topics

### Extending the Firmware

**Custom Application:**
1. Create `robug_app_myapp.py`
2. Implement app class with `async run()` method
3. Integrate into main task loop

**Custom Gait:**
1. Modify `robug_gait.py` trajectory methods
2. Adjust `robug_constants.py` parameters
3. Test with calibrator

**Adding Sensors:**
1. Initialize in `robug_robot.__init__()`
2. Poll in dedicated task
3. Broadcast via message queue

---

## Performance Characteristics

- **Gait Loop**: 8-11ms per cycle
- **Sensor Update**: 250-500ms (4Hz distance, 2Hz battery)
- **Servo Response**: ~1-2ms per command
- **BLE Latency**: ~10-50ms depending on connection
- **Memory**: ~50% of available RAM for typical operation

---

## License

* the [RoBug](https://www.hackster.io/robots4all/robug-tiny-robot-big-attitude-ec32c7) documentation on hackster.io
including videos and images are licensed under [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/deed.en)
* all RoBug hardware design files (blender, pcb, schematic, stl etc.) are licensed under [CERN-OHL-W v2](https://cern-ohl.web.cern.ch/)
* the RoBug software (firmware, MIT App inventor application) is licensed under [GPL v3](https://www.gnu.org/licenses/gpl-3.0.en.html)
* the vl53l0x driver used in this project is based on [MicroPython_VL53L0X](https://github.com/kapetan/MicroPython_VL53L0X) by kapetan and is provided under the MIT license.\
  See tof_sensor.py for details

---

**Document Version:** 1.0  
**Last Updated:** 2026  
**For:** RoBug Firmware - All versions



