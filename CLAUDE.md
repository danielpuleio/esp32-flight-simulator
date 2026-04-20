# CLAUDE.md — ESP32 Flight Simulator Project Context

This file is loaded automatically by Claude Code when running in this repo.
It consolidates cross-session memory so any Claude instance (laptop or WSL)
starts with full project context.

---

## User Profile — Daniel Puleio

- Software engineer at Teralink Technologies (Cambridge, MA), flight software team
- Teralink builds terahertz radios for satellites/drones; Daniel is on FSW, not RF
- ~6.5 years Java background, actively learning C++
- Prior embedded experience: NASA Goddard (C++ radar software), UB Nanosatellite Lab
- GitHub: `danielpuleio`
- Currently on unpaid hiatus from Teralink (~4 weeks remaining, ends ~mid-May 2026)
- Team encouraged this embedded skill-building project during the hiatus
- Career path: left safe job → startup (didn't work out) → pivoted to embedded
- Motivation: AI/job market pressure on pure software; hardware skills are durable
- Prefers ESP-IDF for real project code; Arduino IDE only for hardware verification

## Project — ESP32 Flight Software Simulator

Two-month skill-building project: a satellite-style flight computer simulator
on ESP32. Goal: learn/demonstrate real flight-software patterns (FreeRTOS tasks,
state machines, sensor fusion, fault handling, telemetry).

### Architecture
- FreeRTOS task-based: IMU 100Hz, Baro/Mag 10Hz, state estimator (Kalman),
  flight mode FSM, health monitor/WDT, telemetry, servo control (PID)
- Flight modes: BOOT → SELF_TEST → STANDBY → ARMED → ACTIVE → FAULT → SAFE
- MANUAL mode (servo by uplink) and HEADING_HOLD mode (PID tracks compass)
- Two-ESP32 LoRa architecture: outdoor GPS node → LoRa RF → desk flight sim

### Hardware Status
| Component | Interface | Status |
|---|---|---|
| ESP32 WROOM-32D | — | Working |
| MPU-6050 IMU | I2C 0x68 | Detected, live data verified |
| BME280 baro | I2C 0x76 | Soldered by Daniel, verified |
| QMC5883L mag | I2C 0x0D | Soldered by Daniel, heading tracks |
| All 3 on shared I2C bus | — | Verified 2026-04-17 |
| Servo (Inland 9G) | PWM GPIO13 | In hand, not yet wired |
| LoRa RFM95W | SPI | On order |
| GPS NEO-M8N | UART | On order |

### Pins
- I2C: SDA=GPIO21, SCL=GPIO22
- LoRa SPI: SCK=18, MISO=19, MOSI=23, CS=5, RST=14, DIO0=2
- GPS UART: RX2=16, TX2=17
- Servo: GPIO13

### MPU-6050 axis convention (as mounted on breadboard, verified 2026-04-20)
- +X → right edge of board
- +Y → forward edge of board
- +Z → up (out of chip's top face)
- Right-handed, matches MPU-6050 datasheet. Matters for Madgwick, PID, servo sign.

### MPU-6050 bias at rest (verified 2026-04-20, reproducible across motion test)
- Accel bias: X≈+0.01g, Y≈-0.02g, Z≈+0.07g (Z bias within datasheet ±0.4g spec)
- Gyro bias: X≈-0.5, Y≈+1.8, Z≈-1.8 dps (well inside ±20 dps spec)
- Subtract these in a calibration step before feeding any state estimator.

### Next Milestones
- Sensor fusion (complementary → Madgwick → Kalman) for roll/pitch/yaw
- Servo wiring + PID heading-hold
- WiFi + WebSocket telemetry → browser UI (Three.js aircraft + env gauges)
- LoRa two-node link once radios arrive
- GPS outdoor node

## Dev Environment

### Machines
- Foxtrot-Tango — Windows desktop, WSL2 Ubuntu available
- VIPER — Ubuntu laptop, ESP32 plugged in directly
  - User: daniel, IP: 192.168.4.79 (LAN, dynamic)
  - SSH: key auth only, no password, port 22

### Workflow (Option C — both machines, git sync)
- Laptop is primary for flashing/building (has USB)
- WSL is secondary for edits when laptop is off
- Repo: github.com/danielpuleio/esp32-flight-simulator (public)
- Rule: one primary per session; `git pull` before editing, `git push` at end

### Toolchain on laptop
- Claude Code CLI at ~/.local/bin/claude (native install, 2.1.114+)
- PlatformIO Core 6.1.19 via pip3
- Python 3.12.3

### Still TODO
- Wire servo on PWM GPIO13
- Add sensor library deps to platformio.ini
- Verify port 22 not internet-exposed (check router)
- ssh-agent setup for passphrase caching

## Workflow Preferences

Daniel treats mastering Claude Code as building up agent memory/context across
sessions. He prefers to capture architecture decisions into memory rather than
rushing to execute setup.

How to apply:
- When Daniel describes plans, commit to CLAUDE.md proactively
- Don't push "do it now" if he's in planning mode
- Sessions are cumulative — each should leave the project smarter
- Respect his preference to understand *why* before executing
