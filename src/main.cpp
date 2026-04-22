#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>

#include "wifi_config.h"   // defines WIFI_SSID, WIFI_PASSWORD (gitignored)

// ===== pins =====
static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;

// ===== I2C addresses =====
static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t MAG_ADDR = 0x0D;
static constexpr uint8_t BME_ADDR = 0x76;

// ===== sensor scales =====
static constexpr float ACCEL_LSB_PER_G  = 16384.0f;
static constexpr float GYRO_LSB_PER_DPS = 131.0f;
static constexpr float SEA_LEVEL_HPA    = 1013.25f;

// ===== measured biases (CLAUDE.md, 2026-04-20) =====
static constexpr float ACCEL_BIAS_X = 0.01f;
static constexpr float ACCEL_BIAS_Y = -0.02f;
static constexpr float ACCEL_BIAS_Z = 0.07f;
static constexpr float GYRO_BIAS_X  = -0.5f;
static constexpr float GYRO_BIAS_Y  = 1.8f;
static constexpr float GYRO_BIAS_Z  = -1.8f;

// ===== Madgwick AHRS filter =====
// beta = "trust the accel/mag reference vs the gyro" gain.
// Lower beta = smoother, slower drift correction. Higher = snappier, more jitter.
static constexpr float IMU_DT_S      = 0.01f;
static constexpr float MADGWICK_BETA = 0.1f;
static constexpr float RAD_TO_DEG_F  = 57.29578f;
static constexpr float DEG_TO_RAD_F  = 0.017453293f;

// ===== network =====
static constexpr uint16_t HTTP_PORT = 80;
static constexpr uint16_t WS_PORT   = 81;
static constexpr uint32_t BROADCAST_PERIOD_MS = 20;   // 50 Hz

// ===== magnetometer calibration =====
// Hard-iron offsets (in raw mag counts) subtracted before feeding the filter.
// Heading offset is added to the Madgwick yaw so that user-pointed "north" = 0.
// All four persist in NVS under the "flightsim" namespace.
static constexpr uint32_t CAL_COLLECT_MS = 30000;

enum CalState : uint8_t { CAL_IDLE, CAL_COLLECTING, CAL_AWAITING_NORTH };
static volatile CalState g_cal_state = CAL_IDLE;
static uint32_t g_cal_start_ms = 0;
// Raw-mag running bounds during collection (only written by env_task while
// g_cal_state == CAL_COLLECTING; read by telemetry_task only after the
// state transitions out of COLLECTING, so no mutex needed).
static float g_cal_min_x, g_cal_min_y, g_cal_min_z;
static float g_cal_max_x, g_cal_max_y, g_cal_max_z;

// Applied corrections. Plain floats — 32-bit aligned writes are atomic on
// ESP32 and the UX is "set once, read forever", so no lock required.
static volatile float g_mag_offset_x       = 0.0f;
static volatile float g_mag_offset_y       = 0.0f;
static volatile float g_mag_offset_z       = 0.0f;
static volatile float g_heading_offset_deg = 0.0f;

static Preferences g_prefs;

// ===== shared flight state =====
struct FlightState {
  float roll_deg;       // rotation around forward (Y); right wing down = positive
  float pitch_deg;      // rotation around right (X); nose up = positive
  float yaw_deg;        // rotation around up (Z); from Madgwick, tilt-compensated
  float heading_deg;    // raw (non-tilt-compensated) magnetometer heading, for comparison
  float pressure_hpa;
  float altitude_m;
  float temp_c;
  float mag_x, mag_y, mag_z;   // raw magnetometer, sensor frame, arbitrary units
  uint32_t imu_ticks;
  uint32_t env_ticks;
};

static FlightState       g_state = {};
static SemaphoreHandle_t g_state_mutex;
static SemaphoreHandle_t g_i2c_mutex;

static Adafruit_BME280 bme;
static WebServer        server(HTTP_PORT);
static WebSocketsServer webSocket(WS_PORT);

// ========================================================================
// Embedded browser page. Served once by HTTP on GET /. The page itself
// opens a WebSocket to port 81 for live telemetry.
// ========================================================================

static const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>ESP32 Flight Sim</title>
<style>
  html,body{margin:0;padding:0;height:100%;overflow:hidden;background:#0a1020;
    font-family:ui-monospace,Menlo,Consolas,monospace;color:#cde;}
  #hud{position:fixed;top:12px;left:12px;background:rgba(10,20,40,0.7);
    padding:10px 14px;border:1px solid #234;border-radius:6px;
    line-height:1.6;font-size:14px;min-width:180px;}
  #status{position:fixed;top:12px;right:12px;background:rgba(10,20,40,0.7);
    padding:8px 12px;border:1px solid #234;border-radius:6px;font-size:12px;}
  .k{color:#7af;display:inline-block;width:60px;}
  .v{color:#fff;font-weight:bold;}
  .dot{display:inline-block;width:8px;height:8px;border-radius:50%;
    background:#f55;margin-right:6px;vertical-align:middle;}
  .dot.ok{background:#5f5;}
</style>
</head>
<body>
<div id="hud">
  <div><span class="k">roll</span><span class="v" id="roll">0.0</span>°</div>
  <div><span class="k">pitch</span><span class="v" id="pitch">0.0</span>°</div>
  <div><span class="k">yaw</span><span class="v" id="yaw">0.0</span>°</div>
  <div><span class="k">heading</span><span class="v" id="heading">0.0</span>°</div>
  <div><span class="k">alt</span><span class="v" id="alt">0.0</span> m</div>
  <div><span class="k">temp</span><span class="v" id="temp">0.0</span>°C</div>
</div>
<div id="status"><span class="dot" id="dot"></span><span id="stext">connecting…</span></div>
<script src="https://unpkg.com/three@0.160.0/build/three.min.js"></script>
<script>
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a1020);
  scene.fog = new THREE.Fog(0x0a1020, 18, 80);

  const camera = new THREE.PerspectiveCamera(55, innerWidth/innerHeight, 0.1, 1000);
  camera.position.set(6, 4, 6);
  camera.lookAt(0, 0, 0);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(innerWidth, innerHeight);
  document.body.appendChild(renderer.domElement);

  scene.add(new THREE.AmbientLight(0x88aacc, 0.55));
  const sun = new THREE.DirectionalLight(0xffffff, 1.1);
  sun.position.set(5, 10, 7);
  scene.add(sun);

  const grid = new THREE.GridHelper(40, 40, 0x2a4a70, 0x162a40);
  grid.position.y = -2;
  scene.add(grid);

  // fixed-wing aircraft built from primitives.
  // plane frame: nose at -Z, right wing at +X, up at +Y.
  const plane = new THREE.Group();
  const matBody   = new THREE.MeshStandardMaterial({color:0xdddddd, roughness:0.6});
  const matWing   = new THREE.MeshStandardMaterial({color:0x3366aa, roughness:0.7});
  const matAccent = new THREE.MeshStandardMaterial({color:0xcc3333, roughness:0.7});

  const fuselage = new THREE.Mesh(new THREE.CylinderGeometry(0.2,0.2,3.5,14), matBody);
  fuselage.rotation.x = Math.PI/2;
  plane.add(fuselage);

  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.2,0.6,14), matAccent);
  nose.rotation.x = -Math.PI/2;
  nose.position.z = -2.05;
  plane.add(nose);

  const wings = new THREE.Mesh(new THREE.BoxGeometry(4.6,0.08,0.8), matWing);
  wings.position.z = 0.2;
  plane.add(wings);

  const htail = new THREE.Mesh(new THREE.BoxGeometry(1.6,0.06,0.4), matWing);
  htail.position.z = 1.5;
  plane.add(htail);

  const vtail = new THREE.Mesh(new THREE.BoxGeometry(0.06,0.7,0.5), matWing);
  vtail.position.set(0, 0.3, 1.5);
  plane.add(vtail);

  // apply intrinsic rotations in aircraft order: yaw, then pitch, then roll.
  plane.rotation.order = 'YXZ';
  scene.add(plane);

  function setAttitude(rollDeg, pitchDeg, yawDeg) {
    const d2r = Math.PI/180;
    // signs chosen so the on-screen plane matches sensor tilt:
    //   +roll  = right wing down  -> negative rotation around Three.js Z
    //   +pitch = nose up          -> positive rotation around Three.js X
    //   +yaw   = CW from above. Three.js is right-handed with +Y up, so
    //     positive rotation about +Y is CCW from above — negate to match.
    plane.rotation.z = -rollDeg  * d2r;
    plane.rotation.x =  pitchDeg * d2r;
    plane.rotation.y = -yawDeg   * d2r;
  }

  const statusDot  = document.getElementById('dot');
  const statusText = document.getElementById('stext');
  const el = id => document.getElementById(id);

  let ws;
  function connect() {
    const url = `ws://${location.hostname}:81/`;
    statusText.textContent = 'connecting…';
    statusDot.classList.remove('ok');
    ws = new WebSocket(url);
    ws.onopen  = () => { statusText.textContent = 'connected'; statusDot.classList.add('ok'); };
    ws.onclose = () => {
      statusText.textContent = 'disconnected — retrying';
      statusDot.classList.remove('ok');
      setTimeout(connect, 1000);
    };
    ws.onerror = () => { statusText.textContent = 'error'; };
    ws.onmessage = (ev) => {
      try {
        const s = JSON.parse(ev.data);
        setAttitude(s.roll, s.pitch, s.yaw);
        el('roll').textContent    = s.roll.toFixed(1);
        el('pitch').textContent   = s.pitch.toFixed(1);
        el('yaw').textContent     = s.yaw.toFixed(1);
        el('heading').textContent = s.heading.toFixed(1);
        el('alt').textContent     = s.alt.toFixed(1);
        el('temp').textContent    = s.temp.toFixed(1);
      } catch(e) {}
    };
  }
  connect();

  function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
  }
  animate();

  addEventListener('resize', () => {
    camera.aspect = innerWidth/innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  });
</script>
</body>
</html>)HTML";

// ========================================================================
// Madgwick 9-DOF AHRS update. Sebastian Madgwick (2010), public domain.
// Inputs are in NED body frame: x=forward, y=right, z=down. Gyro in rad/s.
// Accel and mag units don't matter (normalized internally) but must be
// non-zero. Maintains a unit quaternion (q0=w, q1=x, q2=y, q3=z) rotating
// Earth (NED) -> body.
// ========================================================================

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

static void madgwick_update(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float mx, float my, float mz,
                            float dt) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
  float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // gyro integration (rate of change of quaternion from angular velocity)
  qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
  qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
  qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
  qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

  // only apply accel/mag feedback if both vectors are non-zero
  bool have_accel = !(ax == 0.0f && ay == 0.0f && az == 0.0f);
  bool have_mag   = !(mx == 0.0f && my == 0.0f && mz == 0.0f);
  if (have_accel && have_mag) {
    recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
    recipNorm = 1.0f / sqrtf(mx*mx + my*my + mz*mz);
    mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

    _2q0mx = 2.0f * q0 * mx; _2q0my = 2.0f * q0 * my; _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f*q0; _2q1 = 2.0f*q1; _2q2 = 2.0f*q2; _2q3 = 2.0f*q3;
    _2q0q2 = 2.0f*q0*q2; _2q2q3 = 2.0f*q2*q3;
    q0q0=q0*q0; q0q1=q0*q1; q0q2=q0*q2; q0q3=q0*q3;
    q1q1=q1*q1; q1q2=q1*q2; q1q3=q1*q3;
    q2q2=q2*q2; q2q3=q2*q3; q3q3=q3*q3;

    // reference direction of Earth's magnetic field in body frame (projection)
    hx = mx*q0q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1q1 + _2q1*my*q2 + _2q1*mz*q3 - mx*q2q2 - mx*q3q3;
    hy = _2q0mx*q3 + my*q0q0 - _2q0mz*q1 + _2q1mx*q2 - my*q1q1 + my*q2q2 + _2q2*mz*q3 - my*q3q3;
    _2bx = sqrtf(hx*hx + hy*hy);
    _2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0q0 + _2q1mx*q3 - mz*q1q1 + _2q2*my*q3 - mz*q2q2 + mz*q3q3;
    _4bx = 2.0f*_2bx; _4bz = 2.0f*_2bz;

    // gradient-descent correction step
    s0 = -_2q2*(2.0f*q1q3 - _2q0q2 - ax) + _2q1*(2.0f*q0q1 + _2q2q3 - ay) - _2bz*q2*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (-_2bx*q3 + _2bz*q1)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + _2bx*q2*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3*(2.0f*q1q3 - _2q0q2 - ax) + _2q0*(2.0f*q0q1 + _2q2q3 - ay) - 4.0f*q1*(1.0f - 2.0f*q1q1 - 2.0f*q2q2 - az) + _2bz*q3*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (_2bx*q2 + _2bz*q0)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + (_2bx*q3 - _4bz*q1)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0*(2.0f*q1q3 - _2q0q2 - ax) + _2q3*(2.0f*q0q1 + _2q2q3 - ay) - 4.0f*q2*(1.0f - 2.0f*q1q1 - 2.0f*q2q2 - az) + (-_4bx*q2 - _2bz*q0)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (_2bx*q1 + _2bz*q3)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + (_2bx*q0 - _4bz*q2)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1*(2.0f*q1q3 - _2q0q2 - ax) + _2q2*(2.0f*q0q1 + _2q2q3 - ay) + (-_4bx*q3 + _2bz*q1)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (-_2bx*q0 + _2bz*q2)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + _2bx*q1*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

    recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    qDot1 -= MADGWICK_BETA * s0;
    qDot2 -= MADGWICK_BETA * s1;
    qDot3 -= MADGWICK_BETA * s2;
    qDot4 -= MADGWICK_BETA * s3;
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

// Extract Tait-Bryan roll/pitch/yaw (ZYX intrinsic, NED frame) from the
// current quaternion. Roll around X (forward), pitch around Y (right),
// yaw around Z (down) — so positive yaw = CW viewed from above.
static void quat_to_euler_deg(float& roll_deg, float& pitch_deg, float& yaw_deg) {
  float sinp = 2.0f * (q0*q2 - q3*q1);
  if (sinp >  1.0f) sinp =  1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  roll_deg  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG_F;
  pitch_deg = asinf(sinp)                                               * RAD_TO_DEG_F;
  yaw_deg   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG_F;
}

// ========================================================================
// Sensor drivers (all callers must hold g_i2c_mutex during tasks).
// ========================================================================

struct ImuSample  { float ax, ay, az, gx, gy, gz, temp_c; };
struct MagSample  { int16_t x, y, z; float heading_deg; };
struct BaroSample { float temp_c, pressure_hpa, humidity_rh, altitude_m; };

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

static bool baro_init()       { return bme.begin(BME_ADDR, &Wire); }
static BaroSample baro_read() {
  return {
    bme.readTemperature(),
    bme.readPressure() / 100.0f,
    bme.readHumidity(),
    bme.readAltitude(SEA_LEVEL_HPA)
  };
}

// ========================================================================
// Tasks
// ========================================================================

// 100 Hz. Reads the IMU, fuses accel+gyro+mag via Madgwick, publishes
// tilt-compensated roll/pitch/yaw. Uses the latest mag sample written by
// env_task at 10 Hz — stale mag between updates is fine (slow-varying).
static void imu_task(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10);

  for (;;) {
    xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    ImuSample s = imu_read();
    xSemaphoreGive(g_i2c_mutex);

    // snapshot latest mag sample (fed in by env_task)
    float mx_s, my_s, mz_s;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    mx_s = g_state.mag_x;
    my_s = g_state.mag_y;
    mz_s = g_state.mag_z;
    xSemaphoreGive(g_state_mutex);

    // bias correction, still in sensor frame (X=right, Y=forward, Z=up)
    float ax_s = s.ax - ACCEL_BIAS_X;
    float ay_s = s.ay - ACCEL_BIAS_Y;
    float az_s = s.az - ACCEL_BIAS_Z;
    float gx_s = s.gx - GYRO_BIAS_X;
    float gy_s = s.gy - GYRO_BIAS_Y;
    float gz_s = s.gz - GYRO_BIAS_Z;

    // Convert sensor frame -> NED body frame (X=forward, Y=right, Z=down).
    // Accel is negated: Madgwick's reference vector [0,0,1] means gravity
    // direction (down in NED), but the sensor returns specific force which
    // points opposite (up at rest). Mag and gyro are not negated because
    // they measure physical vectors directly. See STUDY_GUIDE Stage 2 §5.
    float ax_n = -ay_s;
    float ay_n = -ax_s;
    float az_n =  az_s;
    float gx_n =  gy_s * DEG_TO_RAD_F;   // gyro also needs dps -> rad/s
    float gy_n =  gx_s * DEG_TO_RAD_F;
    float gz_n = -gz_s * DEG_TO_RAD_F;
    // The QMC5883L breakout on this board is mounted rotated 90° CW (viewed
    // from above) relative to the MPU-6050. So the QMC's own X axis reads
    // the body's backward direction, and the QMC's Y axis reads the body's
    // right. Derivation (verified 2026-04-22):
    //   QMC +X direction == body -Y  =>  body +Y component = -mx_s
    //   QMC +Y direction == body +X  =>  body +X component = +my_s
    //   QMC +Z direction == body +Z  =>  body +Z component = +mz_s
    // Then the standard body(+X right,+Y fwd,+Z up) -> NED(+X fwd,+Y right,+Z down)
    // conversion:
    float mx_n = -mx_s;   // NED forward = body +Y = -QMC_x
    float my_n =  my_s;   // NED right   = body +X = +QMC_y
    float mz_n = -mz_s;   // NED down    = body -Z = -QMC_z

    madgwick_update(gx_n, gy_n, gz_n, ax_n, ay_n, az_n, mx_n, my_n, mz_n, IMU_DT_S);

    float roll_deg, pitch_deg, yaw_deg;
    quat_to_euler_deg(roll_deg, pitch_deg, yaw_deg);
    // Apply user-set "north offset" so that the nose-pointing direction the
    // user called "north" during calibration reads as 0°. This also absorbs
    // magnetic declination as a bonus.
    yaw_deg -= g_heading_offset_deg;
    // wrap to [0, 360)
    while (yaw_deg <    0.0f) yaw_deg += 360.0f;
    while (yaw_deg >= 360.0f) yaw_deg -= 360.0f;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.roll_deg  = roll_deg;
    g_state.pitch_deg = pitch_deg;
    g_state.yaw_deg   = yaw_deg;
    g_state.imu_ticks++;
    xSemaphoreGive(g_state_mutex);

    vTaskDelayUntil(&last_wake, period);
  }
}

static void env_task(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(100);

  for (;;) {
    xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    MagSample  m = mag_read();
    BaroSample b = baro_read();
    xSemaphoreGive(g_i2c_mutex);

    float mx_raw = (float)m.x;
    float my_raw = (float)m.y;
    float mz_raw = (float)m.z;

    if (g_cal_state == CAL_COLLECTING) {
      if (mx_raw < g_cal_min_x) g_cal_min_x = mx_raw;
      if (mx_raw > g_cal_max_x) g_cal_max_x = mx_raw;
      if (my_raw < g_cal_min_y) g_cal_min_y = my_raw;
      if (my_raw > g_cal_max_y) g_cal_max_y = my_raw;
      if (mz_raw < g_cal_min_z) g_cal_min_z = mz_raw;
      if (mz_raw > g_cal_max_z) g_cal_max_z = mz_raw;
    }

    // Apply hard-iron correction before publishing — filter and UI both
    // consume the calibrated values.
    float mx_cal = mx_raw - g_mag_offset_x;
    float my_cal = my_raw - g_mag_offset_y;
    float mz_cal = mz_raw - g_mag_offset_z;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.heading_deg  = m.heading_deg;
    g_state.mag_x        = mx_cal;
    g_state.mag_y        = my_cal;
    g_state.mag_z        = mz_cal;
    g_state.pressure_hpa = b.pressure_hpa;
    g_state.altitude_m   = b.altitude_m;
    g_state.temp_c       = b.temp_c;
    g_state.env_ticks++;
    xSemaphoreGive(g_state_mutex);

    vTaskDelayUntil(&last_wake, period);
  }
}

// Runs a simple three-state calibration state machine alongside the serial
// telemetry print. Commands:
//   c  -> start 30s hard-iron collection (rotate board through all orientations)
//   n  -> capture "north" offset (press when nose points at true/magnetic N)
static void telemetry_task(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(500);

  for (;;) {
    // --- command handling ---
    while (Serial.available()) {
      char c = Serial.read();
      if (c == 'c' && g_cal_state == CAL_IDLE) {
        g_cal_min_x = g_cal_min_y = g_cal_min_z =  1e9f;
        g_cal_max_x = g_cal_max_y = g_cal_max_z = -1e9f;
        g_cal_start_ms = millis();
        g_cal_state = CAL_COLLECTING;
        Serial.println("CAL: rotate board through every orientation for 30s. GO.");
      } else if (c == 'n' && g_cal_state == CAL_AWAITING_NORTH) {
        float yaw_now;
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        yaw_now = g_state.yaw_deg;
        xSemaphoreGive(g_state_mutex);
        // yaw_now already includes old heading offset; add it back to get "uncorrected yaw",
        // then the new offset = uncorrected yaw (which makes current direction read as 0).
        float new_offset = g_heading_offset_deg + yaw_now;
        while (new_offset >= 360.0f) new_offset -= 360.0f;
        while (new_offset <    0.0f) new_offset += 360.0f;
        g_heading_offset_deg = new_offset;
        g_prefs.putFloat("hdg_off", new_offset);
        g_cal_state = CAL_IDLE;
        Serial.printf("CAL: north offset = %.1f deg. Saved.\n", new_offset);
      }
    }

    // --- collection timer ---
    if (g_cal_state == CAL_COLLECTING && millis() - g_cal_start_ms >= CAL_COLLECT_MS) {
      float ox = 0.5f * (g_cal_max_x + g_cal_min_x);
      float oy = 0.5f * (g_cal_max_y + g_cal_min_y);
      float oz = 0.5f * (g_cal_max_z + g_cal_min_z);
      g_mag_offset_x = ox;
      g_mag_offset_y = oy;
      g_mag_offset_z = oz;
      g_prefs.putFloat("mag_ox", ox);
      g_prefs.putFloat("mag_oy", oy);
      g_prefs.putFloat("mag_oz", oz);
      g_cal_state = CAL_AWAITING_NORTH;
      Serial.printf("CAL: hard-iron offset x=%.1f y=%.1f z=%.1f (saved)\n", ox, oy, oz);
      Serial.println("CAL: point nose NORTH (use phone compass), then press 'n'.");
    }

    // --- normal telemetry line ---
    FlightState s;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    s = g_state;
    xSemaphoreGive(g_state_mutex);

    const char* cal_tag =
        g_cal_state == CAL_COLLECTING      ? " [CAL collecting]" :
        g_cal_state == CAL_AWAITING_NORTH  ? " [CAL awaiting N]" : "";

    Serial.printf(
      "roll=% 6.1f pitch=% 6.1f yaw=%5.1f hdg=%5.1f "
      "mag=(%+6.0f,%+6.0f,%+6.0f) alt=%.1fm T=%.1fC "
      "imu=%lu env=%lu ws=%u%s\n",
      s.roll_deg, s.pitch_deg, s.yaw_deg, s.heading_deg,
      s.mag_x, s.mag_y, s.mag_z,
      s.altitude_m, s.temp_c,
      (unsigned long)s.imu_ticks, (unsigned long)s.env_ticks,
      webSocket.connectedClients(), cal_tag);

    vTaskDelayUntil(&last_wake, period);
  }
}

// Blocks until WiFi associates or gives up; runs the HTTP + WS event pumps
// and broadcasts flight state to any connected browser at ~30 Hz.
static void net_task(void*) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setSleep(false);   // disable modem sleep; buffering otherwise stacks 100s of ms of latency
  Serial.printf("WiFi: connecting to \"%s\"", WIFI_SSID);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi: FAILED. UI disabled.");
    vTaskDelete(nullptr);
    return;
  }
  Serial.print("WiFi: connected. Open http://");
  Serial.print(WiFi.localIP());
  Serial.println("/ in a browser.");

  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });
  server.begin();
  webSocket.begin();

  uint32_t next_broadcast = millis();

  for (;;) {
    webSocket.loop();
    server.handleClient();

    uint32_t now = millis();
    if ((int32_t)(now - next_broadcast) >= 0) {
      next_broadcast = now + BROADCAST_PERIOD_MS;

      FlightState s;
      xSemaphoreTake(g_state_mutex, portMAX_DELAY);
      s = g_state;
      xSemaphoreGive(g_state_mutex);

      char buf[224];
      int n = snprintf(buf, sizeof(buf),
        "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"heading\":%.1f,"
        "\"alt\":%.2f,\"temp\":%.1f,\"ts\":%lu}",
        s.roll_deg, s.pitch_deg, s.yaw_deg, s.heading_deg,
        s.altitude_m, s.temp_c, (unsigned long)now);
      if (n > 0) webSocket.broadcastTXT((uint8_t*)buf, (size_t)n);
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ========================================================================
// setup / loop
// ========================================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!imu_init())  { Serial.println("IMU init failed");  while (true) delay(1000); }
  if (!mag_init())  { Serial.println("MAG init failed");  while (true) delay(1000); }
  if (!baro_init()) { Serial.println("BARO init failed"); while (true) delay(1000); }

  g_state_mutex = xSemaphoreCreateMutex();
  g_i2c_mutex   = xSemaphoreCreateMutex();

  // Load persisted mag calibration (zeros if first boot / never calibrated).
  g_prefs.begin("flightsim", false);
  g_mag_offset_x       = g_prefs.getFloat("mag_ox",  0.0f);
  g_mag_offset_y       = g_prefs.getFloat("mag_oy",  0.0f);
  g_mag_offset_z       = g_prefs.getFloat("mag_oz",  0.0f);
  g_heading_offset_deg = g_prefs.getFloat("hdg_off", 0.0f);
  Serial.printf("CAL: loaded offsets mag=(%.1f, %.1f, %.1f) hdg_off=%.1f\n",
    g_mag_offset_x, g_mag_offset_y, g_mag_offset_z, g_heading_offset_deg);
  Serial.println("Type 'c' in the serial monitor to (re)calibrate.");

  // Seed mag in shared state so imu_task has non-zero mag on its first tick.
  // Tasks haven't started yet, so no mutex needed.
  MagSample m0 = mag_read();
  g_state.mag_x = (float)m0.x - g_mag_offset_x;
  g_state.mag_y = (float)m0.y - g_mag_offset_y;
  g_state.mag_z = (float)m0.z - g_mag_offset_z;

  // Sensor/control tasks on core 1 (same as Arduino loop).
  xTaskCreatePinnedToCore(imu_task,       "imu", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(env_task,       "env", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(telemetry_task, "tlm", 4096, nullptr, 1, nullptr, 1);
  // Networking on core 0 (where the WiFi stack lives), larger stack for HTTP/WS.
  xTaskCreatePinnedToCore(net_task,       "net", 8192, nullptr, 1, nullptr, 0);

  Serial.println("All sensors OK. FreeRTOS tasks running.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
