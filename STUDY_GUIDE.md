# Study Guide

A running companion to the code in this repo. Each stage adds a new section
explaining the *concepts* the code in that stage introduces — not a changelog
of what changed, but an answer to "what is this thing, and why is it here."

---

## Stage 1 — FreeRTOS tasks + complementary filter

Before this stage, `loop()` did everything: read IMU, read mag, read baro,
print. Single thread, single rate (10 Hz), no way to run the IMU faster than
the barometer. Stage 1 replaces that with three concurrent tasks sharing
state through mutexes, plus a first attitude estimator.

### 1. Why FreeRTOS tasks instead of `loop()`

Real flight software has jobs that run at *different rates*, and those rates
matter:

- IMU at 100 Hz because attitude changes are fast — at 10 Hz your estimate
  lags a full 100 ms behind reality, which is long enough for a drone to
  flip before you notice.
- Barometer at 10 Hz because pressure changes slowly and the sensor itself
  only updates around that rate.
- Telemetry at 10 Hz because humans don't need more than that.

With a single `loop()` you have exactly one rate. FreeRTOS gives you
*preemptive multitasking*: multiple independent "threads" (tasks) that each
run at their own cadence, and the kernel switches between them on a timer.

**Mental model from Java**: a FreeRTOS task ≈ a `Thread` running a `while
(true) { ... sleep(period) ... }` loop, except the scheduler is cooperative
with hard real-time guarantees and every task has its own fixed-size stack
allocated up front (no heap growth during a task switch).

### 2. Anatomy of a task

```cpp
xTaskCreatePinnedToCore(imu_task, "imu", 4096, nullptr, 3, nullptr, 1);
//                      ^fn       ^name  ^stack         ^prio       ^core
```

- **Function** — a `void fn(void*)` that never returns. The loop inside
  *is* the task.
- **Stack size** — bytes reserved for this task's call stack. 4096 is
  generous for our work; run out and the chip silently corrupts memory.
- **Priority** — higher number preempts lower. IMU=3 beats env=2 beats
  telemetry=1. If the IMU is ready to run, the kernel pauses whatever
  lower-priority task was running and switches to it.
- **Core** — ESP32 has two cores. We pin everything to core 1 (the same
  core Arduino's `loop()` runs on) so all I2C transactions happen on one
  core. Simplest correct choice for now.

Inside each task, the pattern is:

```cpp
TickType_t last_wake = xTaskGetTickCount();
const TickType_t period = pdMS_TO_TICKS(10);   // 10 ms = 100 Hz
for (;;) {
  // ... do work ...
  vTaskDelayUntil(&last_wake, period);
}
```

`vTaskDelayUntil` is the key: it sleeps until a fixed multiple of `period`
after `last_wake`, so your task runs at an *absolute* cadence even if one
iteration takes longer than expected. `vTaskDelay(10)` would give you
"period + work time" drift; `vTaskDelayUntil` corrects for that.

### 3. Shared state and the mutex

Three tasks need to talk to each other. The IMU task produces roll/pitch,
the env task produces heading/altitude/temp, the telemetry task consumes
all of them. The natural shape is a shared struct:

```cpp
struct FlightState { float roll_deg; float pitch_deg; /* ... */ };
static FlightState g_state;
```

But concurrent access without synchronization is a bug. Imagine telemetry
reads `g_state.roll_deg` just as the IMU task is halfway through writing a
new float — you could get a torn value (first two bytes of the new float,
last two of the old). Floats aren't atomic on 32-bit MCUs in the general
case, and multi-field reads definitely aren't.

The fix is a **mutex** (mutual-exclusion lock). Exactly one task at a time
can hold it:

```cpp
xSemaphoreTake(g_state_mutex, portMAX_DELAY);   // block until acquired
g_state.roll_deg  = roll_deg;                   // critical section
g_state.pitch_deg = pitch_deg;
xSemaphoreGive(g_state_mutex);                  // release
```

Rules of thumb:
- **Hold briefly**. Copy in, copy out — don't do I/O or math inside the
  critical section.
- **Always pair take/give**. A dropped `give` permanently stalls every
  other task waiting on it.
- Readers snapshot the whole struct once (copy it into a local
  `FlightState s`) and work with the local copy, so they don't hold the
  lock while printing.

### 4. The I2C bus is also shared state

Easy to miss: the `Wire` bus itself is a shared resource. Two tasks doing
`Wire.beginTransmission(...)` at the same time will scramble each other's
transactions. So we have a *second* mutex, `g_i2c_mutex`, held around every
sensor read. It's the same pattern applied to a different "resource."

Whenever you add a peripheral that multiple tasks touch (SPI bus, a UART,
a hardware timer, a global variable), ask: "who else uses this?" If the
answer isn't "one task, ever," you need synchronization.

### 5. Sensor frame and axis convention

The MPU-6050 on this breadboard (see CLAUDE.md) is oriented:

- **+X** → right edge of the board
- **+Y** → forward edge of the board
- **+Z** → up, out of the chip's top face

That's a right-handed frame. The filter math depends on it, so let's walk
through the signs.

**Accelerometer at rest, level:** the chip measures *specific force*,
which at rest equals the opposite of gravity. Gravity points world-down,
so the accel reads +1 g along +Z. `(ax, ay, az) ≈ (0, 0, +1)`.

**Tilt the right wing down** (positive roll by the aircraft convention we
want):
- The +X axis now points partly downward.
- Gravity has a component along +X, so `ax` becomes positive.
- `az` decreases (gravity no longer aligned with +Z).
- Formula: `accel_roll = atan2(ax, az)` — positive when right wing down. ✓

**Tilt the nose up** (positive pitch):
- The +Y axis now points partly upward.
- Gravity has a component along −Y, so `ay` becomes negative.
- `accel_pitch = atan2(-ay, sqrt(ax² + az²))` — positive when nose up. ✓

**Gyroscope** measures rotation rate in deg/s, right-hand rule around each
body axis:
- `gy` (around forward Y) = roll rate
- `gx` (around right X) = pitch rate
- `gz` (around up Z) = yaw rate (we'll use this when we add mag fusion)

If any of these signs disagree with what the aircraft in the UI does,
revisit this section before changing random minus signs in the filter —
a wrong sign here cascades everywhere.

### 6. Bias calibration

An ideal gyro at rest reads 0 dps. A real MPU-6050 reads something like
+1.8 dps on Y even sitting perfectly still. Integrate +1.8 dps for ten
seconds and your yaw angle is 18° off — from *doing nothing*.

We measured the biases once at rest (CLAUDE.md, 2026-04-20) and subtract
them every sample. Without this step, the gyro integrator in the
complementary filter accumulates error faster than the accel reference
can pull it back, and roll/pitch drift visibly over seconds.

A more robust system re-measures bias every time you boot on a known-level
surface. That's a refinement for later.

### 7. The complementary filter

Each sensor has a failure mode:
- **Gyro**: perfectly responsive to fast motion, but drifts over seconds
  because bias errors accumulate through integration.
- **Accel**: gives a correct *long-term* reference (gravity always points
  down), but any vibration, hand movement, or translation corrupts it.

Notice the dual: gyro is right on short time scales, wrong on long ones;
accel is the opposite. The complementary filter exploits that:

```cpp
angle = α · (angle + gyro_rate · dt)    // high-pass: trust gyro short-term
      + (1 − α) · accel_angle           // low-pass: trust accel long-term
```

With `α = 0.98` at `dt = 0.01 s`:
- Fast motions (a quick tilt) come almost entirely from the gyro integral —
  the 0.02 accel term barely nudges it.
- Over a few seconds of being still, the accel term repeatedly pulls the
  angle back toward the true gravity vector, washing out gyro drift.

The effective time constant is `τ = dt · α/(1−α) ≈ 0.5 s`. That's the
window over which gyro drift gets corrected by accel.

Knobs:
- Raise α → smoother, but slower to correct drift.
- Lower α → corrects drift faster, but accel noise leaks into the output
  as jitter.
- 0.98 is a reasonable starting point; tune while watching the live output.

**Limitations of the complementary filter** (these are why Madgwick and
Kalman exist):
- No yaw. Accel can't measure yaw (rotation around gravity doesn't change
  the gravity vector). Stage 2 adds the magnetometer to handle yaw.
- Assumes the accel reading is gravity. During real acceleration (takeoff,
  hard banking turn) that's wrong, and the filter temporarily trusts the
  wrong thing. Madgwick and Kalman have principled ways to down-weight the
  accel when motion is detected.
- Ad-hoc. There's no formal derivation from sensor noise models — α is a
  knob you tune. Kalman derives the equivalent weight from covariances.

### 8. What to verify on this stage

Flash and watch the serial monitor:

- Roll and pitch read near zero when the board is flat. If they don't,
  either the biases changed or the mount rotated.
- Tilt the right edge down → roll goes positive.
- Tilt the forward edge up → pitch goes positive.
- `imu=N env=M` counters — after 10 seconds you should see roughly
  `imu≈1000, env≈100`. If not, a task isn't running at its period
  (check the i2c mutex — a deadlock or slow sensor read will starve it).
- Let the board sit still for 30 s. Roll and pitch should not drift more
  than a degree or two. If they drift steadily, re-measure gyro biases.

---

## Stage 3 — WiFi, WebSocket telemetry, Three.js UI

Stage 1 gave us attitude over serial. Stage 3 gets that attitude onto a
live 3D aircraft in your browser. The filter is unchanged — this stage is
purely about *transport* (ESP32 → browser) and *visualization*.

### 1. Why WebSocket over the obvious alternatives

We need the ESP32 to push 30 Hz updates to a browser. Options:

| Transport | How it works | Why not |
|---|---|---|
| HTTP polling | Browser asks "any new data?" 30×/sec | Huge per-message overhead (headers + handshake each time); wastes battery and bandwidth |
| Server-Sent Events (SSE) | Server → browser text stream over HTTP | Works, but one-way. WebSocket is same complexity and bidirectional, which helps when Stage 4 adds uplink (arming, mode switches) |
| Raw TCP / UDP | Custom protocol | Browsers can't open raw sockets |
| **WebSocket** | Full-duplex framed messages on top of HTTP upgrade | Single connection, low overhead per frame, bidirectional, browsers speak it natively |

One persistent connection, tiny per-frame overhead (~2–6 bytes of framing),
and you get back-pressure and keep-alive for free.

### 2. What runs on which core

ESP32 has two Xtensa cores. Arduino-ESP32 runs the WiFi stack on core 0 and
`loop()` on core 1. We now pin tasks deliberately:

- **Core 1** (same side as `loop()`): `imu_task`, `env_task`, `telemetry_task`.
  Keeps all I2C work on one core — no cross-core cache issues on the Wire
  bus.
- **Core 0** (WiFi core): `net_task`. Colocates network work with the WiFi
  driver so there's no pointless cross-core thrashing when a WebSocket
  frame goes out.

The IMU task at priority 3 will preempt the net task at priority 1 even
across cores if the scheduler decides to migrate — but by pinning, we
eliminate migration as a variable. This is a common pattern: **sensor and
control on one core, networking on the other.**

### 3. The `net_task` loop

Two jobs in the same loop, on separate rates:

```cpp
for (;;) {
  webSocket.loop();         // service incoming frames / pings
  server.handleClient();    // service incoming HTTP requests

  if (millis() >= next_broadcast) {   // gate outbound broadcast to 30 Hz
    next_broadcast += 33;
    // snapshot state under mutex, format JSON, broadcastTXT
  }

  vTaskDelay(pdMS_TO_TICKS(5));  // yield the core
}
```

The `WebServer` and `WebSocketsServer` libraries are **synchronous** —
they don't run their own thread. If you don't call their `loop()` methods
frequently, frames stall and connections drop. We call them as fast as
possible, and only *broadcast* at 30 Hz using a `millis()` comparator.

### 4. Why `vTaskDelay(5)` and not `vTaskDelayUntil`

Sensor tasks use `vTaskDelayUntil` because their work is bounded and they
must hit an absolute cadence. The net task is different: incoming packets
arrive unpredictably and `webSocket.loop()` can take varying time. We just
want to be responsive, not precise. A plain `vTaskDelay(5ms)` yields the
CPU so lower-priority tasks and the idle task can run (the idle task feeds
the watchdog — never starve it).

### 5. JSON as the wire format

Small, text-based, native to the browser:

```
{"roll":1.23,"pitch":-0.45,"yaw":0.00,"heading":142.3,"alt":12.3,"temp":22.8,"ts":123456}
```

Built with `snprintf` — no library dependency, no allocations. At ~120 bytes
and 30 Hz that's ~30 kbit/s, trivial for WiFi. If/when payload size
matters (LoRa link in later stages), we'll swap to a binary format
(packed struct or CBOR).

**`yaw` is 0 in Stage 3.** Stage 2 (next) will populate it from the
magnetometer. The field is in the JSON now so the UI doesn't have to
change when that data arrives.

### 6. Three.js in one paragraph

Three.js is a WebGL wrapper that gives you a scene graph. The primitives
we use:

- **Scene**: root container, holds everything renderable.
- **Camera**: viewpoint (we use `PerspectiveCamera` with a ~55° FOV).
- **Mesh**: a shape (`Geometry`) + how it looks (`Material`).
- **Group**: a `Mesh` container that can be rotated/translated as one
  unit. We build the aircraft as a `Group` of six meshes (fuselage, nose,
  main wings, horizontal tail, vertical tail), then rotate the Group — all
  children inherit the transform.
- **Light**: `AmbientLight` for fill, `DirectionalLight` for a sun.
- **Renderer**: draws the scene into a `<canvas>` every `requestAnimationFrame`.

Loaded from CDN (`unpkg.com`) so we don't have to host it on the ESP32.

### 7. Mapping sensor frame to Three.js world frame

This is the subtle part. The MPU frame and Three.js world frame don't
line up by default. The plane mesh is built so that:

- Fuselage lies along Three.js **Z**, with nose at **−Z**.
- Main wings span along Three.js **X**, right wing at **+X**.
- Vertical stabilizer points along Three.js **+Y**.

So:
- Sensor **forward** (our +Y) = Three.js **−Z** (nose direction).
- Sensor **right**   (our +X) = Three.js **+X**.
- Sensor **up**      (our +Z) = Three.js **+Y**.

Roll/pitch/yaw mapping (and signs!):

| Sensor concept | Rotates around (sensor frame) | Rotates around (Three.js) | Sign |
|---|---|---|---|
| roll (right wing down = +) | sensor forward (+Y) | Three.js **Z** | **negated** — positive-Z rotation lifts the right wing |
| pitch (nose up = +) | sensor right (+X) | Three.js **X** | direct — positive-X rotation raises the nose (−Z) |
| yaw (CW from above = +) | sensor up (+Z) | Three.js **Y** | direct — positive-Y rotation is CW from above, matching compass heading |

Three.js applies Euler angles in an order you specify via
`plane.rotation.order`. We set `'YXZ'`: yaw first, then pitch, then roll.
That's the aircraft / Tait-Bryan convention — each rotation is intrinsic
to the previous (roll happens in the plane's own reference frame *after*
yaw and pitch have been applied).

If the plane ever moves the *wrong way* for a given tilt, the fix is
almost always in this table, not in the filter math.

### 8. WiFi SSID/password handling

The `include/wifi_config.h` file holds the credentials and is **gitignored**
(see `.gitignore`). A committed template `wifi_config.example.h` shows the
format. Standard pattern for any secret-ish project config:

1. Real values in a gitignored file.
2. Template with placeholders checked in.
3. Documentation in the README when it matters.

Compile-time hardcoding is fine for a dev project. For anything shipped,
you'd move to NVS / preferences (ESP32 has a key-value flash store) or a
provisioning flow (AP mode, captive portal, user types creds in).

### 9. What to verify on this stage

After `pio run -t upload -t monitor`:

1. Serial prints something like:
   ```
   WiFi: connecting to "Luna"...
   WiFi: connected. Open http://192.168.1.42/ in a browser.
   ```
2. Open that URL on a laptop/phone on the *same WiFi network*. The page
   loads dark, with a white/blue aircraft centered and a grid below.
3. Status badge top-right goes green and says "connected".
4. HUD top-left shows roll/pitch/yaw/heading/alt/temp updating 30×/sec.
5. Tilt the breadboard:
   - Right edge down → plane banks right (right wing drops). Roll HUD goes positive.
   - Nose (forward edge) up → plane pitches up. Pitch HUD goes positive.
   - `yaw` stays at 0.0 for now — Stage 2 will turn it on.
6. `heading` HUD changes as you rotate the board, but the *plane* doesn't
   yaw yet. That's the intentional pedagogical gap for Stage 2.

If the plane moves in the wrong direction for a given tilt, revisit the
table in §7 — it's a sign, not a filter, issue.

---

## Stage 2 — 9-DOF Madgwick filter + yaw

Stage 1 gave us roll/pitch from complementary filter. Stage 2 brings the
magnetometer into the loop and upgrades to Madgwick — the algorithm that
actually runs on most hobby flight controllers. The pay-off: yaw comes
alive, and the plane in the browser now tracks heading.

### 1. Why raw compass heading wasn't enough

In Stage 1 we had a `heading_deg` in the JSON, computed as
`atan2(mag_y, mag_x)`. It worked when flat. The moment you tilt the board,
it breaks — and badly.

Why? The magnetometer measures the Earth's magnetic field vector in the
**body frame**. When the board is level, Earth's field has a horizontal
component (points roughly North) and a downward component (the field
"dips" into the ground — roughly 65° down in NYC). Level + North-facing:
`mag_x ≈ 0, mag_y ≈ strong_positive, mag_z ≈ large_negative`. The
`atan2(y, x)` gives heading 0° (North). Good.

Now tilt the board — say, 30° right-wing-down. The body's +X axis now has
a component pointing down, so the strong downward field leaks into
`mag_x`. The `atan2(y, x)` answer swings wildly, even though you're still
facing the same direction. This is why raw compass heading is useless on
a moving aircraft.

### 2. Tilt compensation, two ways

The fix is to **project the mag vector into the Earth's horizontal plane
before computing heading**. You need the current roll and pitch to do it.

Two approaches:

- **Manual tilt compensation**. Use roll/pitch from the complementary
  filter to rotate the mag vector into world frame, then `atan2` the
  horizontal components. Clean, small code. What many 6-DOF + mag builds
  do when they don't want a full AHRS filter.
- **9-DOF AHRS filter** (Madgwick, Mahony, Kalman). Feed all nine raw
  sensor values in — accel + gyro + mag — and the filter produces a
  quaternion that implicitly contains the tilt-compensated heading.

We went with Madgwick because it does both jobs at once (tilt-compensated
yaw and a principled fusion of all three sensors), and it's what real
hobby flight stacks run.

### 3. Quaternions in one page

A quaternion is four numbers `(w, x, y, z)` that represent a 3D rotation.
It's *not* an obscure math toy — it's what you should reach for any time
you need to track orientation. Why:

- **No gimbal lock.** Euler angles (roll/pitch/yaw) have a degenerate
  point at pitch ±90° where two axes collapse into one. Quaternions
  don't. A plane can loop and point straight up without the math blowing
  up.
- **Composable cleanly.** Two rotations multiply together (`q1 * q2`)
  without the order-of-operations pitfalls that plague Euler angles.
- **Smooth interpolation.** SLERP between two quaternions gives a smooth
  shortest-path rotation; interpolating Euler angles gives ugly wobbles.
- **Only 4 numbers with 1 constraint** (unit length). Rotation matrices
  are 9 numbers with 6 constraints.

Intuition: a unit quaternion encodes an axis of rotation `(x, y, z)` and
how much to rotate around it, packed so that `w = cos(θ/2)` and `(x,y,z)`
is the rotation axis scaled by `sin(θ/2)`. You almost never do that
unpacking by hand — you just multiply, normalize, and convert to Euler
at the edges of the system (like when we display roll/pitch/yaw in the
HUD).

In our code, the quaternion lives in the four file-static floats
`q0, q1, q2, q3` (that's `w, x, y, z`). Every Madgwick update nudges it
toward the orientation that best explains the current sensor readings.

### 4. What Madgwick actually does (without the equations)

Each tick (100 Hz) the filter does two things:

1. **Predict** — integrate the gyro rate into the current quaternion.
   This is "assume last orientation + angular velocity × dt = new
   orientation." It's always right for a *brief* moment, but it drifts.
2. **Correct** — compare the predicted orientation's expected gravity
   direction and expected magnetic field direction to what the accel and
   mag actually measured. If they disagree, nudge the quaternion a little
   in the direction that would make them agree. "A little" is controlled
   by the gain `β`.

The math is gradient descent: there's an error function that measures
"how wrong is this quaternion," and each tick we take one small step
down its gradient. Madgwick's contribution was deriving that gradient in
closed form so the whole update is a fixed-size chunk of arithmetic — no
iteration, no matrix inversions. That's why it runs at 100 Hz on a tiny
microcontroller.

Knobs:
- `β = 0.1` (our default). Higher β = faster drift correction, more
  accel/mag noise leaks through to the output. Lower β = smoother, but
  gyro drift persists longer. Same tradeoff as the complementary filter's
  α, but over all three axes simultaneously.
- You can set β=0 and you'll get pure gyro integration (will drift
  visibly in ~10 s). Useful for sanity-checking that bias correction is
  correct.

### 5. Sensor-frame → NED body-frame conversion

The Madgwick formulas come from the standard aerospace convention: body
frame **X = forward, Y = right, Z = down** (NED). Our MPU on the
breadboard is **X = right, Y = forward, Z = up** (see CLAUDE.md).

Two options: re-derive Madgwick in our frame, or convert inputs at the
boundary. Converting at the boundary is way less error-prone:

```
NED.x = sensor.y         (forward   = sensor forward)
NED.y = sensor.x         (right     = sensor right)
NED.z = -sensor.z        (down      = -sensor up)
```

This applies to **any vector in the body frame** — accel, mag, and gyro
angular velocity all transform the same way. The code does exactly this
conversion in `imu_task` before calling `madgwick_update`. If you later
rotate the MPU on the breadboard, this is the *only* place that needs to
change.

One extra thing for gyro: the MPU gives us **degrees per second**, but
Madgwick wants **radians per second**. We multiply by `π/180` in the same
step.

**Accel sign gotcha.** Madgwick's internal reference vector `[0, 0, 1]`
means "gravity direction in the earth frame" — i.e., **down** in NED. But
accelerometers don't measure gravity. They measure *specific force*,
which at rest points in the opposite direction (the reaction force
holding the sensor up). So the accel input must be **negated** before
going into Madgwick — otherwise the filter thinks gravity points up, and
it'll converge to a 180° flipped pose. That's why the conversion looks
like:

```
ax_NED = -sensor.y      (negated!)
ay_NED = -sensor.x      (negated!)
az_NED =  sensor.z      (axis-flip and accel-negate cancel)
```

Gyro and mag are *not* negated — they measure physical quantities (rate,
field vector), not reaction forces. The sign flip is an accel-specific
convention mismatch.

### 6. Getting Euler angles back out

For the browser UI we need roll/pitch/yaw numbers, not a quaternion. The
standard conversion from a unit quaternion to Tait-Bryan angles in the
NED body frame:

```
roll  = atan2( 2(q0·q1 + q2·q3) , 1 − 2(q1² + q2²) )   // around X (fwd)
pitch = asin ( 2(q0·q2 − q3·q1) )                       // around Y (right)
yaw   = atan2( 2(q0·q3 + q1·q2) , 1 − 2(q2² + q3²) )   // around Z (down)
```

These give angles in radians; we multiply by `180/π` for the output.

The signs are correct for our semantic conventions (+roll = right wing
down, +pitch = nose up, +yaw = CW from above) **because** we converted
everything into NED body frame before the filter ran. If you ever see
the wrong sign in the UI, the bug is in §5, not here.

A subtlety: `asin` of values just outside `[-1, +1]` returns NaN. Floating
point can nudge the argument over the edge when pitch is near ±90°. The
code clamps to `[-1, +1]` defensively.

### 7. How the data actually flows at runtime

```
env_task (10 Hz)                      imu_task (100 Hz)
  read mag (I2C)                        read MPU accel+gyro (I2C)
  write mag_x/y/z into g_state  ──┐     read latest mag_x/y/z from g_state
  ... baro, heading ...           │     bias-correct (sensor frame)
                                  │     sensor-frame → NED
                                  │     madgwick_update(...)
                                  │     quat_to_euler_deg(...)
                                  └─>   write roll/pitch/yaw to g_state

                  net_task (50 Hz)
                    snapshot g_state under mutex
                    JSON-encode  →  WebSocket  →  browser  →  Three.js
```

IMU runs 10× faster than mag, so Madgwick sees each mag sample 10 times
before a new one arrives. That's fine — mag is slow-varying, and the
gyro-only prediction between mag updates remains tilt-compensated via
accel.

### 8. Known limitations (things that are still "wrong")

- **Hard-iron offset.** Magnetometers pick up DC bias from nearby metal
  and permanent magnets. A proper calibration rotates the sensor through
  all orientations, measures the offset of the resulting mag-vector
  sphere from the origin, and subtracts it. We're not doing that. Your
  *absolute* heading is probably 20–40° off from true north, but the
  plane yaws *directionally correctly*. The relative motion is what
  matters for the UI.
- **No magnetic declination.** True heading = magnetic heading − local
  declination. Long Island is ~13° West in 2026, so true north is at
  magnetic ~13°. Ignored for now.
- **No accel gating during motion.** If you shake the board, accel gets
  motion-corrupted and Madgwick still trusts it with the full β weight.
  Professional estimators detect "in motion" and temporarily lower β
  (or pause the correction entirely). Not a concern for desk testing.
- **Init transient.** The quaternion starts at (1, 0, 0, 0) = identity
  = aligned with NED. If your first orientation isn't flat-and-north,
  the filter takes 1–2 seconds to swing to the right pose. Expected.

### 9. What to verify on this stage

Re-flash and test with the browser open:

- **Roll / pitch**: still behave correctly (tilt right → plane banks
  right, nose up → plane pitches up). The Madgwick output of these should
  look very close to Stage 1, maybe a touch smoother.
- **Yaw comes alive**: spin the breadboard flat on the desk — the plane
  rotates in the UI. That's the big new thing.
- **Yaw is tilt-independent**: pick a heading, rotate to face it, and
  now tilt the board arbitrarily. The plane's yaw should mostly stay
  put. If it tracked with the tilt, tilt compensation isn't working.
- **Compare `yaw` vs `heading` in the HUD**. `heading` is the raw,
  non-compensated mag reading. When flat they should agree. When
  tilted they should diverge — `yaw` stays sensible, `heading` jumps
  around.
- **Gyro-only sanity**: temporarily set `MADGWICK_BETA = 0.0f` in
  `main.cpp`, re-flash. The plane should track fast motion perfectly but
  drift visibly over ~10 s. Then restore 0.1 and watch the accel/mag
  correction pull it back.

---

## Stage 2.5 — Magnetometer calibration

Madgwick yaw tracks *relative* motion even on a noisy mag, but *absolute*
heading is off by an arbitrary amount because of hard-iron offset
(permanent ferromagnetic material rigidly attached to the sensor —
including, in our case, the breadboard's nickel-plated steel spring
clips). This stage adds a one-command calibration to fix that.

### 1. The geometry of hard-iron offset (why single-point doesn't work)

If you sweep an ideal magnetometer through every orientation while
holding it still in space, the readings trace a **sphere** centered at
the origin, with radius equal to the local field magnitude.

Hard-iron adds a constant DC vector to every reading. The sphere stops
being centered at the origin — it's **shifted** by that vector. If you
project the 3D sphere onto the horizontal plane, you get a circle that's
no longer centered at (0, 0).

Why this matters: heading comes from `atan2(my, mx)`. For that to give
uniform angles as you spin the sensor, the circle traced by `(mx, my)`
must be origin-centered. If it's shifted:

- "North" might read the right-ish angle, but...
- "East" won't be 90° clockwise from north. It might be 60° or 120°.
- The angular error *varies with heading* — worst case ~2×arctan(offset/radius).

A single-point "I'm facing north, record the offset" fix subtracts a
single number from the output. That fixes **one** heading, not the whole
circle. Everywhere else is still wrong.

**Rotation calibration fixes the whole circle.** Once the circle is
re-centered, a separate single-point "this way is north" step cleans up
any remaining rotational offset (including the ~13° magnetic declination
for NYC). That's why the code does both.

### 2. The min-max method (our algorithm)

Full-blown calibration fits a 3D ellipsoid to the collected data (handles
soft-iron too). We're using the simpler "min-max" method, which fits a
sphere (hard-iron only):

```
offset_x = (max_x + min_x) / 2
offset_y = (max_y + min_y) / 2
offset_z = (max_z + min_z) / 2
```

That's the center of the axis-aligned bounding box of the cloud of
samples — a rough but usable estimate of the sphere's center. Good enough
for ~80% of the accuracy gain. To make it work well:

- **Rotate through every orientation** you can. Flip the board upside
  down too — you need the top and bottom of the sphere, not just a
  horizontal sweep.
- **Keep the rotation slow-ish** — very fast spins bias the samples
  toward a few orientations (you pass through others too briefly).
- **Do it in the same location** the board will live in. Moving near a
  different set of ferromagnetic stuff (different desk, different lab)
  invalidates the calibration.

### 3. The north offset

After the hard-iron subtraction, the Madgwick-computed yaw points in
*some* direction that it thinks is north — influenced by mounting
alignment of the mag chip and magnetic declination. We bypass trying to
model these separately and instead let the user say "you're facing north
right now."

The code captures the current post-offset yaw and stores it as
`heading_offset_deg`. On every future update, the output yaw is
`raw_yaw − heading_offset`, wrapped into `[0, 360)`. Side effect: this
also absorbs the 13° NYC magnetic declination with no extra code.

### 4. Persistence with NVS / Preferences

ESP32 has a flash-backed key-value store called **NVS** (non-volatile
storage). Arduino-ESP32 wraps it as the `Preferences` class:

```cpp
g_prefs.begin("flightsim", false);                    // namespace, RW
float v = g_prefs.getFloat("mag_ox", 0.0f);           // default on miss
g_prefs.putFloat("mag_ox", value);                    // write (wear-leveled)
```

This survives reboot and reflash (as long as you don't erase the whole
flash). Calibrate once, reboot, values stick. Great match for config-ish
data. Not great for high-frequency writes — each write has some erase
overhead, and flash has a finite write endurance (~100k cycles per
sector).

### 5. Usage

On any boot, the ESP32 prints the loaded offsets:

```
CAL: loaded offsets mag=(0.0, 0.0, 0.0) hdg_off=0.0
Type 'c' in the serial monitor to (re)calibrate.
```

To calibrate:

1. In the serial monitor, type `c` and press Enter.
2. For 30 seconds, rotate the board through every orientation — flips,
   yaws, tilts. Imagine drawing every point on the surface of a sphere.
3. The ESP32 prints the computed offsets and prompts for north.
4. Point the nose at a known north reference (your phone compass), type
   `n`, press Enter.
5. Offsets are saved to NVS. Re-calibrate any time by pressing `c` again.

Every telemetry line during calibration is tagged `[CAL collecting]` or
`[CAL awaiting N]` so you can tell what state the machine is in.

### 6. Hard-iron vs. soft-iron (and why the breadboard is the wrong mount)

The min-max calibration above addresses exactly one failure mode. There's
a second mode it can't touch, and our current mount inflicts both.

**Hard-iron distortion** is a constant magnetic offset that rides along
with the sensor: a permanent magnet glued to the chip, a DC-carrying
trace, a magnetized screw. Whatever the sensor's orientation, the same
extra vector `b` adds to every reading:

```
m_measured = m_true + b
```

The fix is a vector subtract. Sphere fit, done. This is what our
calibration handles.

**Soft-iron distortion** is caused by *unmagnetized* ferromagnetic
material near the sensor — steel, iron, nickel. It isn't a permanent
magnet, but in the presence of Earth's field its magnetic domains line
up and it temporarily becomes one, adding its own contribution to what
the sensor measures. The critical property: the induced field points in
a direction that depends on *how the ambient field is oriented relative
to the material*. Rotate the sensor, and the steel's contribution
rotates with it — the distortion is orientation-dependent, not constant.

Mathematically, instead of a simple offset, you get a linear transform:

```
m_measured = A · m_true + b
```

where `A` is a 3×3 matrix that stretches and rotates the true vector.
Geometrically, the cloud of readings as you sweep through all
orientations is an **ellipsoid**, not a sphere — stretched along some
axes, tilted in the sensor frame. The min-max algorithm fits a sphere
and averages over the ellipsoid, which corrects the center but leaves
the shape distortion in place. A full ellipsoid fit (Merayo's method,
or iterative least-squares) solves it — that's ~100 lines of linear
algebra and we're skipping it for now.

**Why the breadboard is a bad mount.** Cheap breadboards use
nickel-plated **steel** springs as their row contacts. There are dozens
of them within a centimeter or two of the QMC5883L sitting on top. Put
a neodymium magnet on the breadboard and you can feel the pull — that's
a direct hands-on demonstration that there's a significant ferromagnetic
mass rigidly attached to the sensor.

Three things go wrong at once:

- **Hard-iron component** — the clips carry some residual magnetization
  from manufacturing and tool handling. This part our sphere fit *can*
  correct.
- **Soft-iron component** — the unmagnetized portion of the steel gets
  induced-magnetized by Earth's field, producing a direction-dependent
  warp our sphere fit *cannot* correct.
- **Non-uniform field** — the clips aren't a single point source; they're
  distributed under the whole board. The field they produce varies over
  the sensor's footprint. Even a proper ellipsoid fit (which assumes a
  spatially uniform `A`) degrades here.

The three combine to make absolute heading wrong by a
heading-dependent amount that no static calibration can clean up.
**Relative yaw** — tracking how much the board rotated — still works
fine, because short-timescale motion is gyro-dominated and the mag
error stays roughly consistent while the board sits still.

**Practical policy:**

- On the breadboard, don't bother running calibration. The hard-iron
  piece we could correct is dwarfed by the uncorrectable soft-iron mess.
  Trust **relative yaw**; treat absolute heading as decorative.
- Once the sensor moves to a non-ferrous mount — plastic standoff,
  perfboard with brass rivets, a real PCB with the mag kept away from
  steel — run calibration there. Hard-iron will be small enough that the
  sphere fit is a reasonable approximation, and soft-iron should drop
  below the noise floor.
- Recalibrate after any physical change: new chassis, new mounting
  screws, moved to a different workbench. The hard-iron vector is a
  property of "sensor + everything ferrous attached to it," so it's
  invalidated any time that assembly changes.
- Motors, servo PWM wires, and battery current loops produce
  **time-varying** fields that no static calibration addresses. If you
  ever build an airframe, the mag should physically live far from those
  sources — typically on a short boom above the fuselage. That's a
  Stage-many-later problem.


