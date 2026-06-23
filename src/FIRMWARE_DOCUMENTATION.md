# RoBug Firmware Documentation

## Overview

RoBug firmware is a MicroPython-based control system for an affordable, palm-sized quadruped robot. The firmware implements inverse kinematics, gait generation, motion control, Bluetooth Low Energy remote control, and a small app framework for experimental behaviours. This document describes the firmware architecture, the main modules under src/, and the runtime behaviour introduced/changed by the most recent firmware updates.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Core Components](#core-components)
3. [Initialization and Main Loop](#initialization-and-main-loop)
4. [Gait System](#gait-system)
5. [Inverse Kinematics](#inverse-kinematics)
6. [Hardware Control](#hardware-control)
7. [Communication (BLE)](#communication-ble)
8. [Motion Controller Improvements](#motion-controller-improvements)
9. [Application Framework](#application-framework)
10. [Configuration](#configuration)

---

## Architecture Overview

The firmware follows a modular, asynchronous architecture using MicroPython's `asyncio` (uasyncio) primitives. Multiple concurrent tasks handle different aspects of robot control so sensor polling, BLE I/O and motion control can run without blocking each other.

Key design principles:
- Modular components for Gait, IK, motion controller and hardware abstractions
- Asynchronous tasks for non-blocking real-time operation
- Hardware abstraction via `robug_constants.py`
- Per-unit calibration stored in `robug_calibration.json`

---

## Core Components

Files in `src/` and their responsibilities (summary):

- `main.py` — application entrypoint, task orchestration (BLE server, sensor tasks, motion controller and supervisor tasks)
- `robug_robot.py` — Robot model: legs, sensors (VL53L0X), LEDs; loads calibration and exposes high-level robot APIs
- `robug_leg.py` — Per-leg logic (gait generator, IK invocation, joint commands)
- `robug_gait.py` — Gait generator (trotting trajectories, push support)
- `robug_ik.py` — Inverse kinematics solver for the 2-DOF leg
- `robug_joints.py` — Servo mapping, angle → PWM conversion, calibration application
- `robug_ctrl.py` — Supervisor-side client API used by high-level tasks to send motion commands
- `robug_mocon.py` — Motion controller (rbmocon) — orchestrates gait loop, animations and scripted actions
- `robug_ble.py` — BLE server (rbble) using aioble; command reception and sensor notifications
- `robug_com.py` — Internal command translation layer used by motion controller
- `robug_utils.py` — Utility classes (v3 vector, helpers)
- `tof_sensor.py` — VL53L0X time-of-flight distance sensor driver (MIT licensed upstream)
- `robug_calibration.json` — Per-unit servo calibration data

---

## Initialization and Main Loop

main.py initialisation sequence (high-level):
1. Instantiate BLE server: `ble = rbble()`
2. Create robot instance and load calibration: `r = robug()`
   - robot initialisation creates VL53L0X instance and sets up LEDs
3. Setup ADC for battery monitoring
4. Create motion controller and supervisor queues
5. Start asyncio tasks:
   - `ble.msg_handler()` — BLE advertising/connection handler
   - `serve_sensor_data()` — distance & battery polling (see section below)
   - `pulse_leds()` — LED animator
   - `m.run()` — motion controller main loop
   - `fpv_rc(ble)` — supervisor remote-control state machine

Concurrent tasks are coordinated using asyncio tasks and simple queues (deque) for command/reply passing.

---

## Gait System

RoBug uses a trotting gait where diagonal leg pairs move together (FL+RR vs RL+FR). The gait generator implements two main phases per leg: support and swing. Configuration parameters in `robug_constants.py` allow tuning of:
- loop timing (`_GAIT_LOOP_TIME`), support and swing durations (`_GAIT_SUPPORT_TICKS`, `_GAIT_SWING_TICKS`)
- stride (`_GAIT_HALF_STRIDE`), swing amplitude (`_GAIT_SWING_AMPL`), height (`_GAIT_HEIGHT`)
- push support (`_GAIT_PUSH_STRENGTH`) and overlap behaviour

The gait generator exposes methods to increment the gait counter, produce per-step x/z trajectories and enables/disables push support.

---

## Inverse Kinematics

A 2-link analytical IK solver (law of cosines) computes femur and tibia joint angles for the requested foot Cartesian positions in leg space. The solver uses constants for link lengths (`_L_FEMUR`, `_L_TIBIA`) defined in `robug_constants.py`.

---

## Hardware Control

Servo control:
- PWM frequency and pulse ranges implemented by `robug_joints.py` are applied to servos via the platform `PWM` API
- Calibration offsets and per-servo gains are read from `robug_calibration.json` by `robug_robot.create_robug()` and applied when converting angles to PWM ticks

Sensors:
- VL53L0X distance sensor driver is provided in `tof_sensor.py`. It is based on a MicroPython driver and is distributed under the MIT license (see header of `tof_sensor.py`).
- Touch sensors wired to configurable GPIOs provide simple digital input for top (pet) and bottom (collision) detection
- Battery voltage is sampled using ADC and scaled with the configured voltage divider factor

LEDs:
- Green and red status LEDs use PWM for pulsing effects; convenience methods are exposed on the robot instance (`set_brightness_grn`, `set_brightness_red`).

---

## Communication (BLE)

The BLE server (`rbble` in `robug_ble.py`) was updated in the merged firmware PR. Key details:

- Uses `aioble` + `bluetooth` (MicroPython BLE stack) and `asyncio` for asynchronous BLE handling
- Service UUID: defined in `rbble.SERVICE_UUID` (random UUID used by this project)
- Characteristic layout:
  - Command/Write characteristic (CHAR_UUID1) — write-only; created with `write_no_response=True` to avoid blocking mobile apps
  - Distance notify characteristic (CHAR_UUID2) — read & notify
  - Battery state-of-charge notify characteristic (CHAR_UUID3) — read & notify
- Behaviour:
  - `msg_handler()` advertises and accepts connections, then runs three tasks while connected: `send_data_dist`, `send_data_soc`, and `receive_cmd`
  - `send_data_dist(connection)` notifies the current distance value every ~500 ms (struct-packed little-endian uint32)
  - `send_data_soc(connection)` notifies the SOC (battery) value; current implementation uses `await asyncio.sleep(2)` (2 second interval)
  - Commands are received via the write characteristic and handled by `handle_command_rc()` (maps single-byte codes to movement strings like FWD/BWD/LEFT/RIGHT/STOP) or `handle_command_raw()` when mode is switched
- Command / data model:
  - `rbble.cmd` holds the current high-level command string (e.g. 'FWD', 'STOP') used by the supervisor
  - `rbble.dist` and `rbble.soc` are updated by the sensor task (`serve_sensor_data()`) before being notified to the connected client

Note: the BLE implementation uses packed little-endian uint32 values (`struct.pack('<I', value)`) for notifications to simplify parsing on the mobile side.

---

## Motion Controller Improvements

The motion controller (`rbmocon` in `robug_mocon.py`) received a number of functional updates in the recent firmware PR:

- Added helper rotation functions:
  - `rotate_point_center(p, theta)` and `rotate_point_point(p, o, a)` for rotating foot position vectors. The latter converts angles to radians internally and supports rotating around an arbitrary origin.
- `rotate_body(theta)` implements body rotation using incremental steps and updates IK + joints at each sub-step so body rotation appears smooth and incremental. The method computes foot position vectors relative to the rotation pivot, rotates them in small increments and updates servos between steps.
- Animation and scripted actions (start/stop animations, sit/stand, purr, attack, turning left/right) remain available and are driven by `set_positions_relative()` in the `robug` API. The updated APIs keep the same high-level message/queue driven approach (see `robug_com.py` / `rbcom.get_command()`).
- `run()` continues to be the central async loop: it interprets translated commands from the queue, runs animations or gait loop updates and drives `r.inc_loop_counters()`, `r.calculate_foot_positions()`, `r.solve_ik()` and `r.set_joints()` while `bRunLoop` is enabled.

---

## Application Framework

The firmware provides a minimal app framework: place an app module named `robug_app_<name>.py` in `src/` implementing a class with an `async run()` method to be executed from the main loop. Built-in example apps include:
- `robug_app_fpv.py` — camera / FPV streaming helper
- `robug_app_explorer.py` — autonomous exploration using distance sensor
- `robug_app_calibrator.py` — interactive servo calibration tool
- `robug_app_template.py` — example template to implement additional behaviours

---

## Configuration

Local per-device overrides can be provided via `robug_local_config.py` (gitignored). Use `robug_local_config.py.example` as a template.
Hardware-specific constants (pin mappings, I2C bus, gait defaults) are centralized in `robug_constants.py`.

Calibration data lives in `robug_calibration.json` and is loaded at robot creation time. The calibrator app writes this file after interactive calibration.

---

## Runtime Sensor Behaviour (important)

- Distance sensor (VL53L0X): `serve_sensor_data()` collects a median of 6 samples via `get_distance()` and writes the median distance to `ble.dist`. The task runs approximately every 250 ms (4 Hz) and pushes values to the BLE notify characteristic.
- Battery monitor: sampled via ADC and scaled to a multiple of 10 mV; `serve_sensor_data()` updates `ble.soc` and notifications run on a separate BLE notify task.

---

## Debugging and Testing

Utility scripts:
- `robug_led_test.py` — LED tests
- `robug_app_calibrator.py` — run on the device to adjust `robug_calibration.json` offsets

Common issues:
- Servo jitter: increase `_GAIT_LOOP_TIME`, verify calibration offsets and gains
- Walking instability: adjust `_GAIT_HEIGHT`, `_GAIT_PUSH_STRENGTH` and `_GAIT_SWING_AMPL`
- BLE latency: check polling intervals and queue sizes, ensure mobile app uses non-blocking writes

---

## File Reference

(See top of `src/` for concrete file list)

---

## Licenses / Credits

- RoBug firmware: GPL v3 (see headers in source files)
- VL53L0X driver in `tof_sensor.py` is derived from a MicroPython VL53L0X implementation and is provided under the MIT license (see header in `tof_sensor.py`).

---

**Document Version:** 1.1
**Last Updated:** 2026-06-20
**For:** RoBug Firmware - All versions
