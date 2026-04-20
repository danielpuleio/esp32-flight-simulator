# ESP32 Flight Simulator

A satellite-style flight-computer simulator running on an ESP32. Study project
for embedded flight-software patterns: FreeRTOS tasks, flight-mode state
machine, sensor fusion, fault handling, and telemetry.

## Hardware

| Component        | Interface | Role                                        |
|------------------|-----------|---------------------------------------------|
| ESP32 WROOM-32D  | —         | Flight computer                             |
| MPU-6050         | I2C 0x68  | 3-axis accelerometer + 3-axis gyroscope     |
| BME280           | I2C 0x76  | Pressure / temperature / humidity           |
| QMC5883L         | I2C 0x0D  | 3-axis magnetometer                         |
| Inland 9G servo  | PWM       | Control surface (planned)                   |
| RFM95W           | SPI       | LoRa telemetry link (planned)               |
| u-blox NEO-M8N   | UART      | GPS, outdoor node (planned)                 |

### Pinout

- I2C: SDA = GPIO21, SCL = GPIO22
- LoRa SPI: SCK=18, MISO=19, MOSI=23, CS=5, RST=14, DIO0=2
- GPS UART2: RX=16, TX=17
- Servo PWM: GPIO13

## Planned architecture

FreeRTOS task-based flight loop:

- `imu_task` @ 100 Hz — MPU-6050 sampling, state estimator input
- `env_task` @ 10 Hz — barometer and magnetometer
- `estimator_task` — attitude filter (complementary → Madgwick → Kalman)
- `fsm_task` — flight-mode state machine (`BOOT → SELF_TEST → STANDBY → ARMED → ACTIVE → FAULT → SAFE`)
- `control_task` — servo PID (MANUAL and HEADING_HOLD modes)
- `health_task` — watchdog, fault detection, telemetry

Two-node LoRa topology:

- Outdoor GPS node: ESP32 + NEO-M8N + RFM95W, transmits position fixes
- Desk flight computer: ESP32 + IMU/baro/mag + RFM95W, fuses GPS with onboard sensors

## Build and flash

PlatformIO Core required.

```bash
pio run -t upload        # build and flash
pio device monitor       # serial console at 115200 baud
```

## Status

- [x] PlatformIO toolchain verified on Linux
- [x] I2C bus verified — all three sensors enumerate
- [x] MPU-6050 driver (register-level), axis convention and bias characterized
- [x] QMC5883L driver (register-level), heading output
- [x] BME280 driver (Bosch compensation via Adafruit library)
- [x] Combined 10 Hz telemetry reading all three sensors
- [ ] FreeRTOS task split and shared state block
- [ ] Attitude estimator
- [ ] Servo wiring and PID heading-hold
- [ ] WiFi + WebSocket telemetry, browser 3D UI
- [ ] LoRa link
- [ ] GPS outdoor node
