#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;

static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t MAG_ADDR = 0x0D;
static constexpr uint8_t BME_ADDR = 0x76;

static constexpr float ACCEL_LSB_PER_G   = 16384.0f;
static constexpr float GYRO_LSB_PER_DPS  = 131.0f;
static constexpr float SEA_LEVEL_HPA     = 1013.25f;

struct ImuSample {
  float ax, ay, az;
  float gx, gy, gz;
  float temp_c;
};

struct MagSample {
  int16_t x, y, z;
  float heading_deg;
};

struct BaroSample {
  float temp_c;
  float pressure_hpa;
  float humidity_rh;
  float altitude_m;
};

static Adafruit_BME280 bme;

static void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

static bool imu_init() {
  if (readReg(MPU_ADDR, 0x75) != 0x68) return false;
  writeReg(MPU_ADDR, 0x6B, 0x00);
  delay(100);
  return true;
}

static ImuSample imu_read() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  int16_t tr = (Wire.read() << 8) | Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  return {
    ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G,
    gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS,
    tr / 340.0f + 36.53f
  };
}

static bool mag_init() {
  if (readReg(MAG_ADDR, 0x0D) != 0xFF) return false;
  writeReg(MAG_ADDR, 0x0A, 0x80);
  delay(10);
  writeReg(MAG_ADDR, 0x0B, 0x01);
  writeReg(MAG_ADDR, 0x09, 0x0D);
  delay(10);
  return true;
}

static MagSample mag_read() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, (uint8_t)6);
  int16_t x = Wire.read() | (Wire.read() << 8);
  int16_t y = Wire.read() | (Wire.read() << 8);
  int16_t z = Wire.read() | (Wire.read() << 8);
  float heading = atan2f((float)y, (float)x) * 180.0f / PI;
  if (heading < 0) heading += 360.0f;
  return {x, y, z, heading};
}

static bool baro_init() {
  return bme.begin(BME_ADDR, &Wire);
}

static BaroSample baro_read() {
  return {
    bme.readTemperature(),
    bme.readPressure() / 100.0f,
    bme.readHumidity(),
    bme.readAltitude(SEA_LEVEL_HPA)
  };
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!imu_init())  { Serial.println("IMU init failed");  while (true) delay(1000); }
  if (!mag_init())  { Serial.println("MAG init failed");  while (true) delay(1000); }
  if (!baro_init()) { Serial.println("BARO init failed"); while (true) delay(1000); }

  Serial.println("All sensors OK. Streaming at 10 Hz.");
}

void loop() {
  ImuSample i  = imu_read();
  MagSample m  = mag_read();
  BaroSample b = baro_read();

  Serial.printf(
    "A=[% .2f % .2f % .2f]g  G=[% 6.1f % 6.1f % 6.1f]dps  "
    "hdg=%5.1f  T=%.1fC  P=%.1fhPa  alt=%.1fm\n",
    i.ax, i.ay, i.az, i.gx, i.gy, i.gz,
    m.heading_deg, b.temp_c, b.pressure_hpa, b.altitude_m);

  delay(100);
}
