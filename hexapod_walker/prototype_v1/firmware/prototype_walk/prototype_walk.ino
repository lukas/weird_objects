/*
  Prototype hexapod -- standalone tripod walker
  ----------------------------------------------

  Unlike prototype_servo_bridge.ino (which only relays poses sent over
  serial from a Pi/laptop), THIS sketch generates the gait on the Arduino
  itself.  Power it up, give it ~1.5 s to settle into stance, and the
  robot walks forward on its own -- no host computer required.

  The gait is a direct C port of mujoco_prototype.TripodGait + the
  2-link leg IK (_leg_ik).  Angle sign conventions, joint order, the
  PCA9685 routing, the per-axis safe limits and the EEPROM trim table
  are all identical to prototype_servo_bridge.ino, so a leg calibrated
  with the bridge firmware ('T' commands) walks correctly here too.

  Joint order (matches the sim and docs):
      joint = leg * 3 + axis        leg = 0..5
      axis 0 = yaw, axis 1 = hip pitch, axis 2 = knee pitch
  Legs sit at azimuth (leg + 0.5) * 60 deg = 30, 90, ... , 330 deg.
  Forward (+vx) drives the chassis along its +X axis (the flat edge
  between leg 5 and leg 0).

  PCA9685 routing (one board per 3 legs -- 9 servos each, so every
  leg's 3 servos stay on ONE board and the V+ current splits evenly):
      legs 0..2 (joints  0..8 ) -> board 0x40 channels 0..8
      legs 3..5 (joints  9..17) -> board 0x41 channels 0..8
      channel = (joint < 9) ? joint : joint - 9
      per-board channel = (leg % 3) * 3 + axis

  Serial (115200, newline-terminated) -- optional, for safety/tuning:
      g            go / resume walking
      s            stop (hold current stance, feet planted)
      c            centre all joints (0 deg) and stop
      v <vx> <vy> <omega>   set body velocity (mm/s, mm/s, rad/s)
      T <joint> <deg>       set + persist a per-joint trim (EEPROM)
      P            print the trim table
      ?            help

  Power note:
      Servo V+ must come from the external 5-6 V BEC / bench rails.  Do
      NOT power DS3225 servos from the Arduino 5 V pin.
*/

#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1(0x40);
Adafruit_PWMServoDriver pwm2(0x41);

// --- Servo output scaling + safe limits (mirror servo bridge) -------
const float PWM_MIN_US = 500.0;    // DS3225 nominal -90 deg
const float PWM_MAX_US = 2500.0;   // DS3225 nominal +90 deg
const int   SERVO_HZ   = 50;

const float YAW_LIMIT_LO_DEG  = -35.0;
const float YAW_LIMIT_HI_DEG  =  35.0;
const float HIP_LIMIT_LO_DEG  = -80.0;
const float HIP_LIMIT_HI_DEG  =  30.0;
const float KNEE_LIMIT_LO_DEG = -20.0;
const float KNEE_LIMIT_HI_DEG =  80.0;

// --- Trim persistence (EEPROM) -- same layout as the bridge ---------
const uint16_t EE_MAGIC      = 0x4831;  // 'H','1'
const int      EE_MAGIC_ADDR = 0;
const int      EE_TRIM_ADDR  = 4;
const float    TRIM_LIMIT_DEG = 30.0;

float trim_deg[18] = {0};

// Last commanded angle per joint (post-clamp, pre-trim), so pose
// transitions (stand up / stretch out) can ease from wherever the
// legs currently are instead of snapping.
float target_deg[18] = {0};

// --- Leg geometry (mm) -- mirror hexapod_prototype.py ---------------
const float COXA  =  25.0;
const float FEMUR =  75.0;   // Jun 2026: shortened from 90 (coxa-link crack fix)
const float TIBIA = 130.0;
const float LEG_RADIAL = 100.0;            // chassis apothem (200 mm flat-to-flat / 2)
const float STANCE_HIP_DEG  = -25.0;       // STANCE_FEMUR_DEG
const float STANCE_KNEE_DEG =  60.0;       // STANCE_TIBIA_DEG

// --- Gait parameters (mirror mujoco_prototype.TripodGait defaults) --
// All scale factors are held at 1.0 (scalar lift/stride/period/stance).
float g_period = 0.65;      // s  -- full gait cycle
float g_lift   = 25.0;      // mm -- peak swing foot lift
float g_ramp   = 0.35;      // s  -- ease-in time for stride + lift
float g_vx     = 80.0;      // mm/s forward (chassis +X)
float g_vy     = 0.0;       // mm/s lateral (chassis +Y)
float g_omega  = 0.0;       // rad/s yaw rate

const float PHASE_OFFSET = (float)HALF_PI;
const unsigned long SETTLE_MS = 1500;      // hold stance before walking

// --- Precomputed per-leg constants ----------------------------------
float leg_cos[6], leg_sin[6];
float foot_neutral_z;       // mm, leg-local foot height at stance
float foot_radius_eff;      // mm, body-frame neutral foot radius

// --- Gait state ------------------------------------------------------
float g_phase   = 0.0;      // rad, 0..2pi
float g_elapsed = 0.0;      // s since walking started (drives ramp)
bool  g_walking = false;
unsigned long boot_ms      = 0;
unsigned long last_step_us = 0;
const unsigned long STEP_PERIOD_US = 20000UL;   // 50 Hz control update

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline void axisLimits(int joint_idx, float& lo, float& hi) {
  int axis = joint_idx % 3;
  if (axis == 0)      { lo = YAW_LIMIT_LO_DEG;  hi = YAW_LIMIT_HI_DEG;  }
  else if (axis == 1) { lo = HIP_LIMIT_LO_DEG;  hi = HIP_LIMIT_HI_DEG;  }
  else                { lo = KNEE_LIMIT_LO_DEG; hi = KNEE_LIMIT_HI_DEG; }
}

static inline int trimEepromAddr(int joint_idx) {
  return EE_TRIM_ADDR + joint_idx * (int)sizeof(float);
}

void saveTrim(int joint_idx) {
  if (joint_idx < 0 || joint_idx >= 18) return;
  EEPROM.put(trimEepromAddr(joint_idx), trim_deg[joint_idx]);
  uint16_t magic = EE_MAGIC;
  EEPROM.put(EE_MAGIC_ADDR, magic);
}

void loadTrims() {
  uint16_t magic = 0;
  EEPROM.get(EE_MAGIC_ADDR, magic);
  if (magic != EE_MAGIC) {
    for (int i = 0; i < 18; ++i) {
      trim_deg[i] = 0.0;
      EEPROM.put(trimEepromAddr(i), trim_deg[i]);
    }
    EEPROM.put(EE_MAGIC_ADDR, (uint16_t)EE_MAGIC);
    return;
  }
  for (int i = 0; i < 18; ++i) {
    float v = 0.0;
    EEPROM.get(trimEepromAddr(i), v);
    if (v != v) v = 0.0;  // NaN guard
    trim_deg[i] = clampf(v, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
  }
}

// Clamp to the safe workspace, add the persisted trim, convert to a
// servo pulse and push it to the right PCA9685 channel.  Identical to
// the servo bridge's writeJoint().
void writeJoint(int joint_idx, float angle_deg) {
  if (joint_idx < 0 || joint_idx >= 18) return;
  float lo, hi;
  axisLimits(joint_idx, lo, hi);
  angle_deg = clampf(angle_deg, lo, hi);
  target_deg[joint_idx] = angle_deg;
  float corrected = clampf(angle_deg + trim_deg[joint_idx], lo, hi);
  float us = PWM_MIN_US + (corrected + 90.0) / 180.0 * (PWM_MAX_US - PWM_MIN_US);
  // Legs 0..2 (joints 0..8) -> 0x40 ch 0..8; legs 3..5 (joints 9..17)
  // -> 0x41 ch 0..8.  Keeps each leg's 3 servos on one board.
  Adafruit_PWMServoDriver& drv = (joint_idx < 9) ? pwm1 : pwm2;
  int chan = (joint_idx < 9) ? joint_idx : (joint_idx - 9);
  drv.writeMicroseconds(chan, (int)(us + 0.5));
}

void writeStancePose() {
  for (int i = 0; i < 6; ++i) {
    writeJoint(i * 3 + 0, 0.0);
    writeJoint(i * 3 + 1, STANCE_HIP_DEG);
    writeJoint(i * 3 + 2, STANCE_KNEE_DEG);
  }
}

void centreAll() {
  for (int i = 0; i < 18; ++i) writeJoint(i, 0.0);
}

// Blocking, eased interpolation of all 18 joints from their current
// target angles to `dest` over `dur_ms`.  Uses a raised-cosine
// (ease-in / ease-out) profile so servos start and stop gently.  Used
// for the stand-up / stretch-out pose transitions, which are slow,
// one-shot moves -- the real-time gait loop never calls this.
void sweepTo(const float dest[18], unsigned long dur_ms) {
  float start[18];
  for (int i = 0; i < 18; ++i) start[i] = target_deg[i];
  unsigned long t0 = millis();
  for (;;) {
    unsigned long el = millis() - t0;
    float u = (dur_ms == 0) ? 1.0f : (float)el / (float)dur_ms;
    if (u > 1.0f) u = 1.0f;
    float s = 0.5f * (1.0f - cosf((float)PI * u));   // smoothstep
    for (int i = 0; i < 18; ++i) {
      writeJoint(i, start[i] + (dest[i] - start[i]) * s);
    }
    if (u >= 1.0f) break;
    delay(15);
  }
}

// Ease every leg from wherever it is into the standing stance
// (0, -25, +60) -- i.e. lift the femurs and fold the knees so the
// feet plant below the body.  Stops the gait first.
void standUp() {
  g_walking = false;
  float dest[18];
  for (int i = 0; i < 6; ++i) {
    dest[i * 3 + 0] = 0.0;
    dest[i * 3 + 1] = STANCE_HIP_DEG;
    dest[i * 3 + 2] = STANCE_KNEE_DEG;
  }
  sweepTo(dest, 1200);
}

// Ease every leg out to the dead-straight, horizontal zero pose
// (0, 0, 0) -- all links colinear, pointing straight out.
void stretchOut() {
  g_walking = false;
  float dest[18] = {0};
  sweepTo(dest, 1200);
}

// 2-link planar IK in the leg's yaw frame (port of _leg_ik).  Inputs in
// mm; outputs hip pitch p and knee pitch k in RADIANS.  Returns false
// when the target is out of reach (caller falls back to stance).
bool legIK(float target_x, float target_z, float& p, float& k) {
  float u = target_x - COXA;
  float w = -target_z;
  float L = sqrtf(u * u + w * w);
  if (L > FEMUR + TIBIA - 1e-3f) return false;
  if (L < fabsf(FEMUR - TIBIA) + 1e-3f) return false;
  float cos_k = (L * L - FEMUR * FEMUR - TIBIA * TIBIA) / (2.0f * FEMUR * TIBIA);
  cos_k = clampf(cos_k, -1.0f, 1.0f);
  k = acosf(cos_k);
  p = atan2f(w, u) - atan2f(TIBIA * sinf(k), FEMUR + TIBIA * cosf(k));
  return true;
}

void initGaitConstants() {
  for (int i = 0; i < 6; ++i) {
    float a = (i + 0.5f) * (float)PI / 3.0f;   // leg azimuth
    leg_cos[i] = cosf(a);
    leg_sin[i] = sinf(a);
  }
  float p  = radians(STANCE_HIP_DEG);
  float pt = radians(STANCE_HIP_DEG + STANCE_KNEE_DEG);
  float foot_neutral_x = COXA + FEMUR * cosf(p) + TIBIA * cosf(pt);
  foot_neutral_z       = -FEMUR * sinf(p) - TIBIA * sinf(pt);
  foot_radius_eff      = LEG_RADIAL + foot_neutral_x;
}

// One control update: advance the gait phase by dt seconds and drive
// all 18 joints.  Direct port of TripodGait._foot_target_in_body +
// TripodGait.desired (scalar scales = 1, no command smoothing).
void stepGait(float dt) {
  g_elapsed += dt;
  g_phase = fmodf(g_phase + 2.0f * (float)PI * dt / fmaxf(g_period, 0.05f),
                  2.0f * (float)PI);
  float ramp_amp = fminf(g_elapsed / g_ramp, 1.0f);
  float t_eff = fmaxf(g_period, 0.05f);

  for (int i = 0; i < 6; ++i) {
    float ca = leg_cos[i], sa = leg_sin[i];
    int tripod = (i % 2 == 0) ? 0 : 1;
    float phi = fmodf(g_phase + PHASE_OFFSET + tripod * (float)PI,
                      2.0f * (float)PI);

    float prog, dz;
    if (phi < (float)PI) {                 // swing: lift + carry forward
      float s = phi / (float)PI;
      prog = -0.5f + s;
      dz = g_lift * ramp_amp * sinf((float)PI * s);
    } else {                               // stance: planted, push back
      float s = (phi - (float)PI) / (float)PI;
      prog = 0.5f - s;
      dz = 0.0f;
    }

    float v_x_at = g_vx - g_omega * foot_radius_eff * sa;
    float v_y_at = g_vy + g_omega * foot_radius_eff * ca;
    float dx = prog * v_x_at * t_eff / 2.0f * ramp_amp;
    float dy = prog * v_y_at * t_eff / 2.0f * ramp_amp;

    // Foot target in the chassis (body) frame, then into the leg's
    // yaw frame to split off the yaw angle from the planar reach.
    float fx_b = foot_radius_eff * ca + dx;
    float fy_b = foot_radius_eff * sa + dy;
    float rx = fx_b - LEG_RADIAL * ca;
    float ry = fy_b - LEG_RADIAL * sa;
    float x_yaw =  ca * rx + sa * ry;
    float y_yaw = -sa * rx + ca * ry;
    float yaw_angle = atan2f(y_yaw, x_yaw);
    float r_planar  = sqrtf(x_yaw * x_yaw + y_yaw * y_yaw);

    float p, k;
    if (legIK(r_planar, foot_neutral_z + dz, p, k)) {
      writeJoint(i * 3 + 0, degrees(yaw_angle));
      writeJoint(i * 3 + 1, degrees(p));
      writeJoint(i * 3 + 2, degrees(k));
    } else {
      writeJoint(i * 3 + 0, 0.0);
      writeJoint(i * 3 + 1, STANCE_HIP_DEG);
      writeJoint(i * 3 + 2, STANCE_KNEE_DEG);
    }
  }
}

void startWalking() {
  g_walking = true;
  g_phase = 0.0;
  g_elapsed = 0.0;          // re-arm the ease-in ramp
  last_step_us = micros();
}

void stopWalking() {
  g_walking = false;
  writeStancePose();
}

void printHelp() {
  Serial.println(F("OK prototype_walk (standalone tripod gait)"));
  Serial.println(F("  u            stand up (ease from stretched-out to stance)"));
  Serial.println(F("  z            stretch out flat (ease to the 0/0/0 zero pose)"));
  Serial.println(F("  g            go / resume walking"));
  Serial.println(F("  s            stop, hold stance"));
  Serial.println(F("  c            centre all joints (0 deg) + stop"));
  Serial.println(F("  v <vx> <vy> <omega>   body velocity (mm/s, mm/s, rad/s)"));
  Serial.println(F("  T <joint 0..17> <trim_deg>   (saved to EEPROM)"));
  Serial.println(F("  P            print trim table"));
  Serial.println(F("  ?            help"));
}

void printTrims() {
  Serial.print(F("TRIM"));
  for (int i = 0; i < 18; ++i) { Serial.print(' '); Serial.print(trim_deg[i], 2); }
  Serial.println();
}

void handleLine(char* line) {
  while (*line == ' ') ++line;
  char cmd = line[0];
  char* p = line + 1;

  if (cmd == '?' || cmd == 'H' || cmd == 'h') { printHelp(); return; }
  if (cmd == 'u' || cmd == 'U') { standUp();    Serial.println(F("OK stand")); return; }
  if (cmd == 'z' || cmd == 'Z') { stretchOut(); Serial.println(F("OK stretch")); return; }
  if (cmd == 'g' || cmd == 'G') { startWalking(); Serial.println(F("OK go")); return; }
  if (cmd == 's' || cmd == 'S') { stopWalking();  Serial.println(F("OK stop")); return; }
  if (cmd == 'c' || cmd == 'C') { stopWalking(); centreAll(); Serial.println(F("OK centre")); return; }
  if (cmd == 'P') { printTrims(); return; }
  if (cmd == 'v' || cmd == 'V') {
    g_vx = atof(p);
    while (*p == ' ') ++p; while (*p && *p != ' ') ++p;
    g_vy = atof(p);
    while (*p == ' ') ++p; while (*p && *p != ' ') ++p;
    g_omega = atof(p);
    Serial.print(F("OK v ")); Serial.print(g_vx, 2); Serial.print(' ');
    Serial.print(g_vy, 2); Serial.print(' '); Serial.println(g_omega, 3);
    return;
  }
  if (cmd == 'T') {
    int joint = atoi(p);
    while (*p == ' ') ++p; while (*p && *p != ' ') ++p;
    float deg = atof(p);
    if (joint < 0 || joint >= 18) { Serial.println(F("ERR joint")); return; }
    trim_deg[joint] = clampf(deg, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
    saveTrim(joint);
    Serial.print(F("OK T ")); Serial.print(joint); Serial.print(' ');
    Serial.println(trim_deg[joint], 2);
    return;
  }
  Serial.println(F("ERR unknown"));
}

void pollSerial() {
  static char buf[96];
  static int n = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[n] = '\0';
      if (n > 0) handleLine(buf);
      n = 0;
    } else if (n < (int)sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      n = 0;
      Serial.println(F("ERR line too long"));
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);     // fast-mode I2C so 18 channel writes fit in 20 ms
  pwm1.begin();
  pwm2.begin();
  pwm1.setPWMFreq(SERVO_HZ);
  pwm2.setPWMFreq(SERVO_HZ);
  delay(50);

  loadTrims();               // restore calibration before any motion
  initGaitConstants();
  centreAll();               // power up in the dead-straight zero pose
  printHelp();

  boot_ms = millis();
  last_step_us = micros();
}

void loop() {
  pollSerial();

  // Auto-start (one-shot): hold the stretched-out zero pose briefly,
  // then stand up and begin walking on its own.  Latched so a later
  // 's' / 'z' / 'u' command isn't immediately overridden.
  static bool auto_started = false;
  if (!auto_started && (millis() - boot_ms) >= SETTLE_MS) {
    auto_started = true;
    standUp();
    startWalking();
    Serial.println(F("OK walking"));
  }

  if (g_walking) {
    unsigned long now = micros();
    unsigned long elapsed = now - last_step_us;   // wraps cleanly (unsigned)
    if (elapsed >= STEP_PERIOD_US) {
      last_step_us = now;
      stepGait((float)elapsed * 1e-6f);
    }
  }
}
