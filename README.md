# RoBug
RoBug is a small quadruped robot that combines simplicity, affordability and advanced
robotics features under one roof. RoBug was designed to be both a beginner-friendly
project for everybody who wants to dive into robotics without spending a fortune on
parts and tools, and an open, modular platform for advanced robotics experiments.

Learn more on Hackster: [robug - tiny robot, big attitude](https://www.hackster.io/robots4all/robug-tiny-robot-big-attitude-ec32c7)

<img width="800" height="600" alt="RoBug_and_his_double" src="https://github.com/user-attachments/assets/dab777c2-4b4e-4f69-9bd3-a20df38d0b9e" />

## Firmware documentation

The firmware is implemented in MicroPython and lives in the `src/` directory. It
implements inverse kinematics, gait generation, motion control, Bluetooth remote
control and a small app framework.

The full firmware documentation (living document) is maintained in the repository:

- `src/FIRMWARE_DOCUMENTATION.md` — detailed firmware architecture, runtime behaviour and developer notes.

Quick links:
- Firmware documentation: https://github.com/defstone/RoBug/blob/main/src/FIRMWARE_DOCUMENTATION.md

## What's new (recent firmware changes)

This README was updated to reflect the recent firmware changes merged into `main`.
Key highlights from the merged firmware update:

- BLE server improvements (see `src/robug_ble.py`):
  - Three-characteristic layout (command write + distance notify + battery notify).
  - Non-blocking write characteristic (`write_no_response=True`) to improve mobile app responsiveness.
  - Distance and SOC values are notified periodically (packed little-endian uint32).

- Motion controller updates (`src/robug_mocon.py`):
  - New body-rotation helpers and an incremental `rotate_body()` routine for smooth rotations.
  - Animations and scripted actions (stand, sit, purr, attack, turns) are kept and improved for smoother transitions.

- Sensor and driver notes:
  - VL53L0X driver is provided in `src/tof_sensor.py` and is based on an MIT-licensed MicroPython implementation. See the driver header for details.

For the full technical details please read `src/FIRMWARE_DOCUMENTATION.md`.

Commit with docs update: https://github.com/defstone/RoBug/commit/be2ac065ed6ba4076b18f4a99d399d671a125198

## Overview (high level)

The firmware follows a modular, asynchronous architecture using MicroPython's
`asyncio` (uasyncio) primitives. Tasks run concurrently for:

- BLE handling (advertising, connection, notifications, command handling)
- Sensor polling (distance sensor and battery sampling)
- Motion controller (gait loop, animations, scripted actions)
- Optional application modules (FPV, explorer, calibrator)

Main source files (brief):

- `src/main.py` — entry point, task orchestration
- `src/robug_robot.py` — robot model and hardware glue
- `src/robug_leg.py` — per-leg logic
- `src/robug_gait.py` — gait generator
- `src/robug_ik.py` — inverse kinematics
- `src/robug_joints.py` — servo mapping / PWM
- `src/robug_mocon.py` — motion controller
- `src/robug_ble.py` — BLE server (aioble)
- `src/tof_sensor.py` — VL53L0X time-of-flight driver
- `src/robug_calibration.json` — servo calibration data

## License

- RoBug firmware and software: GPL v3 (see file headers)
- VL53L0X driver in `src/tof_sensor.py` is derived from an MIT-licensed MicroPython implementation — see the file header for details.
- Hardware, documentation and assets: see repository root and Hackster page for individual license information.

---

**Last Updated:** 2026-06-20
