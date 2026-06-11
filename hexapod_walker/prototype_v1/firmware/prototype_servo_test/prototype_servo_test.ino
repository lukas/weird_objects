/*
  Prototype hexapod servo TEST / bring-up sweeper
  ------------------------------------------------

  Walks through all 18 servos ONE AT A TIME and gently sweeps each one so
  you can confirm every motor is alive, wired to the channel you think it
  is, and moving the expected direction.  This is the sketch to flash for
  Stage C / Stage D bench bring-up (see ../WIRING.md) -- it does the same
  job as typing `J <joint> <deg>` by hand in the servo bridge, but it
  marches through the whole robot automatically and tells you which joint
  it is poking before it moves.

  Hardware (identical to prototype_servo_bridge.ino):
      Arduino Mega + 2x PCA9685 (0x40, 0x41).
      One board per 3 legs (each leg's 3 servos stay on ONE board):
        legs 0..2 (joints 0..8)  -> board 0x40 channels 0..8
        legs 3..5 (joints 9..17) -> board 0x41 channels 0..8
        per-board channel = (leg % 3) * 3 + axis
      joint = leg * 3 + axis;  axis 0 = yaw, 1 = hip pitch, 2 = knee pitch

  Power: servo V+ comes from the external 5-6 V BEC / bench rail into the
  PCA screw terminals -- NEVER from the Arduino 5 V pin.  Bench-test with a
  current-limited supply and the legs clamped / feet in the air.

  STARTS IDLE: out of reset it CENTRES all joints and then WAITS, doing
  nothing (onboard LED slow-blinking) until you send a command.  Send `G`
  to begin the hands-free auto sweep (joint 0 -> 17 -> 0 ... sweeping each
  one, LED on while a joint moves), or N/L/A/ENTER to test a single joint
  or leg.  Send any character to stop the auto loop and take manual
  control again.  Motion starts GENTLE -- the sweep defaults to slow speed
  (`S` toggles slow/fast).

  Whole-body moves (all eased, ports of prototype_walk.ino):
      `U` raises every leg up over the body, `P` eases into the walking
      stance, and `W [mm/s]` walks forward using the real tripod gait
      (default 35 mm/s, e.g. `W 20`).  Any other key (e.g. `C`) stops.

  Board note: builds for the Arduino Mega 2560 (the project's target) AND
  the Arduino UNO Q.  On the UNO Q the sketch runs on the STM32 MCU and
  USB serial is tunnelled through the onboard Linux processor, so it uses
  the Arduino_RouterBridge `Monitor` object automatically (install that
  library from the Library Manager).  On the UNO Q the Serial Monitor in
  *Arduino App Lab* is the reliable one (the classic IDE monitor is often
  blank) -- but you don't need it: auto mode moves the motors regardless.

  --- Serial control (optional; Monitor @ App Lab, newline-terminated) ---

      <any key>    STOP the auto loop, take manual control
      <ENTER>      test the NEXT joint (advances 0,1,2,...,17,0,...)
      N <j>        test joint <j> only (0..17), e.g.  N 5
      L <leg>      test all 3 joints of one leg (leg 0..5), e.g.  L 0
      A            test ALL 18 joints in sequence (full sweep tour)
      G            GO: start / resume the hands-free auto loop
      R            repeat the joint you just tested
      C            centre every joint (all to 0 deg) and stop
      X            turn OFF all motors -- cut PWM so servos go limp
      F <0.1..1.0> set sweep amplitude as a fraction of each axis range
      S            slow / fast toggle for the sweep speed
      ?            print this help

  --- Whole-body poses + walking (eased, ported from prototype_walk.ino) ---

      U            raise every leg UP over the body ("legs over its head")
      P            ease into the walking stance / position (feet planted)
      W [mm/s]     walk forward (live tripod gait) at the given speed,
                   default 35 mm/s, e.g.  W 20 ;  negative walks backward.
                   Any key / C stops.

  Each joint test sweeps:  0 deg -> -side -> +side -> back to 0, slowly,
  staying inside the SAFE per-axis limits (same limits as the bridge /
  MuJoCo model), scaled by the amplitude fraction (default 0.6).

  Trims:
      Per-joint trims saved by the servo bridge live in EEPROM and are
      LOADED here too, so "0 deg" matches your mounted-horn calibration.
      This sketch never WRITES trims -- use the bridge's `T` command for
      that.
*/

#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

// EEPROM trim persistence is AVR-only (the Mega's real EEPROM).  Boards
// like the Arduino UNO Q (Zephyr/STM32 core) have no <EEPROM.h>, so we
// guard the include and fall back to in-RAM zero trims there.  Calibrate
// horns on those boards with the bridge's `T` command at run time (or
// just mount horns accurately) -- this TEST sketch never writes trims.
#if defined(__AVR__)
#include <EEPROM.h>
#define HAVE_EEPROM 1
#endif

// --- Console abstraction --------------------------------------------
// On a classic AVR Mega, the USB Serial Monitor is the `Serial` object.
// On the Arduino UNO Q (Zephyr/STM32 core) the sketch runs on the STM32
// MCU and `Serial` is wired to UART pins 0/1 -- NOT to USB.  To reach
// the USB Serial Monitor / App Lab you must use the `Monitor` object from
// the Arduino_RouterBridge library, which tunnels text through the
// onboard Linux processor.  `CONSOLE` picks the right one at compile
// time so the rest of the sketch is identical on both boards.
#if defined(ARDUINO_ARCH_ZEPHYR)
#include <Arduino_RouterBridge.h>
#define CONSOLE Monitor
#define CONSOLE_IS_BRIDGE 1
#else
#define CONSOLE Serial
#endif

// The UNO Q bridge can reorder/garble many small back-to-back writes
// (known arduino-router bug), so every message is assembled into one
// String and pushed with a SINGLE write via say().
static inline void say(const String& s) { CONSOLE.println(s); }

Adafruit_PWMServoDriver pwm1(0x40);
Adafruit_PWMServoDriver pwm2(0x41);

// Which PCA9685 boards actually ACK on the bus (index 0 = 0x40 legs 0-2,
// index 1 = 0x41 legs 3-5).  Probed at boot and on every `I` rescan.  We
// NEVER send I2C to an absent board: a broken/half-connected driver (e.g.
// a cracked 0x41) can wedge the shared bus, after which even writes to the
// healthy 0x40 hang -- that's the "C/X run, then nothing works" failure.
bool have_board[2] = { false, false };

// Whether each present board has been initialised (begin + 50 Hz) since it
// last appeared.  A PCA9685 powers up in SLEEP with no PWM output, so a
// board that wasn't connected at boot (e.g. mid-rewire) would otherwise sit
// dead -- I2C-visible but emitting no pulses, so servos stay limp even with
// V+ present.  We (re)initialise a board the first time it's detected, at
// boot OR on a later `I` scan, so connect-after-boot and hot-plug just work.
// (initBoard() itself is defined below, once SERVO_HZ is in scope.)
bool board_inited[2] = { false, false };

// --- EEPROM trim layout (must match prototype_servo_bridge.ino) -----
const uint16_t EE_MAGIC      = 0x4831;  // 'H','1'
const int      EE_MAGIC_ADDR = 0;
const int      EE_TRIM_ADDR  = 4;
const float    TRIM_LIMIT_DEG = 30.0;

// --- Remembered standing foot height (the `D` descend command) --------
// Foot Z (mm, leg yaw-frame; MORE NEGATIVE = foot further BELOW the hip =
// legs extended further DOWN toward the ground).  `D` walks this value
// down in small steps until you say the feet have touched, then remembers
// it; the stance pose (`P`) and the walk (`W`) then plant the feet at this
// height.  On AVR (Mega) it is persisted right after the trim block; the
// UNO Q (Zephyr/STM32) has no EEPROM, so there it is remembered for the
// session only -- bake a permanent value in with DEFAULT_STAND_Z below.
const uint16_t EE_STAND_MAGIC      = 0x535A;  // 'S','Z'
const int      EE_STAND_MAGIC_ADDR = EE_TRIM_ADDR + 18 * (int)sizeof(float);  // 76
const int      EE_STAND_ADDR       = EE_STAND_MAGIC_ADDR + (int)sizeof(uint16_t);  // 78
// Uncomment + set (mm) to hardcode the standing foot Z on EEPROM-less
// boards (UNO Q).  Leave undefined to use the geometric stance height.
// #define DEFAULT_STAND_Z -150.0
const float DESCEND_STEP_MM = 0.5;   // default foot drop per step (`D <mm>` overrides)
const int   DESCEND_SWEEP_MS = 120;  // ease time for each step
const int   DESCEND_HOLD_MS  = 120;  // dwell after each step so you can watch/feel it

// As the feet descend, the stance pulls them INWARD (smaller radial
// distance from the hip).  Two reasons: (1) geometry -- at the sprawled
// neutral radius (~213 mm) the leg runs out of IK reach only ~107 mm
// below the hip, nowhere near enough to lift the body; (2) torque -- a
// foot 190 mm out from the hip out-levers the servos, so even reachable
// poses can't lift the robot.  The foot radial tapers linearly from the
// neutral sprawl (foot_neutral_x at foot_neutral_z) down to
// STAND_TUCK_X_MM by the time the feet are STAND_TUCK_Z_MM below the
// hip, which keeps the knee bent in a strong, crouched-insect posture.
const float STAND_TUCK_Z_MM = -150.0;  // depth at which feet are fully tucked
const float STAND_TUCK_X_MM = 135.0;   // tucked foot radial (yaw frame, mm)

const float PWM_MIN_US = 500.0;    // DS3225 nominal -90 deg
const float PWM_MAX_US = 2500.0;   // DS3225 nominal +90 deg

// Per-axis SAFE workspace (axis = joint % 3; 0=yaw, 1=hip, 2=knee).
// MUST match prototype_servo_bridge.ino / mujoco_prototype._leg_xml.
const float YAW_LIMIT_LO_DEG  = -35.0;
const float YAW_LIMIT_HI_DEG  =  35.0;
const float HIP_LIMIT_LO_DEG  = -80.0;
const float HIP_LIMIT_HI_DEG  =  30.0;
const float KNEE_LIMIT_LO_DEG = -20.0;
const float KNEE_LIMIT_HI_DEG =  80.0;
const int   SERVO_HZ = 50;

float trim_deg[18]   = {0};
float current_deg[18] = {0};   // last commanded angle, for smooth ramps

// Test sequencer state.
int   g_joint      = 0;    // joint that <ENTER> / R will act on next
float g_amplitude  = 0.6;  // fraction of each axis range to exercise
const int STEP_FAST_MS = 14;  // ms per 1-deg step, brisk
const int STEP_SLOW_MS = 32;  // ms per 1-deg step, gentle (the default)
int   g_step_ms    = STEP_SLOW_MS;  // start SLOW; `S` toggles slow<->fast
bool  g_seen_input = false; // becomes true on the first command received
// Auto mode marches through every joint on its own.  It starts OFF so the
// robot does NOTHING out of reset -- send `G` to begin the sweep.  Any
// other command takes manual control; `G` (re)starts the auto loop.
bool  g_auto       = false;
int   g_auto_gap_ms = 700; // pause between joints in auto mode

// --- Whole-body pose / walk support (ported from prototype_walk.ino) -----
// Beyond the per-joint sweep, the sketch can strike whole-body poses and
// run the real tripod gait.  `g_walk` drives the live gait in loop(); it
// is mutually exclusive with g_auto.
bool  g_walk = false;

// Leg geometry (mm) + stance angles -- mirror hexapod_prototype.py.
const float COXA  =  25.0;
const float FEMUR =  75.0;   // Jun 2026: shortened from 90 (coxa-link crack fix)
const float TIBIA = 130.0;
const float LEG_RADIAL      = 100.0;   // chassis apothem (flat-to-flat / 2)
const float STANCE_HIP_DEG  = -25.0;
const float STANCE_KNEE_DEG =  60.0;

// Gait parameters -- slow, gentle defaults for bench bring-up.
float g_period = 0.90;     // s  -- full gait cycle (slow cadence)
float g_lift   = 22.0;     // mm -- peak swing foot lift
float g_ramp   = 0.45;     // s  -- ease-in for stride + lift
float g_vx     = 35.0;     // mm/s forward (slow); chassis +X
float g_vy     = 0.0;      // mm/s lateral
float g_omega  = 0.0;      // rad/s yaw

#ifndef HALF_PI
#define HALF_PI 1.5707963267948966
#endif
const float PHASE_OFFSET = (float)HALF_PI;

// Precomputed per-leg constants (filled by initGaitConstants()).
float leg_cos[6], leg_sin[6];
float foot_neutral_x, foot_neutral_z, foot_radius_eff;
// Standing foot Z actually used by the stance pose + gait baseline.  Set
// from geometry (or DEFAULT_STAND_Z / EEPROM) at boot, then refined live by
// the `D` descend command and remembered.
float g_stand_z = 0.0f;

// Forward declarations (definitions appear further down).
bool legIK(float target_x, float target_z, float& p, float& k);
float stanceFootX(float foot_z);
bool stanceDestAtZ(float foot_z, float dest[18]);
void descendLegs(float step_mm);

// Live gait state.
float g_phase = 0.0, g_elapsed = 0.0;
unsigned long last_step_us = 0;
const unsigned long STEP_PERIOD_US = 20000UL;   // 50 Hz control update

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Wrap an angle into [0, 2pi).  Used instead of fmodf() because the
// Zephyr/newlib-nano fmodf pulls in __errno (unresolved in this build).
// The gait phase only advances by small steps, so a subtract-wrap is
// exact here and avoids any libm modulo dependency.
static inline float wrap2pi(float x) {
  const float T = 2.0f * (float)PI;
  while (x >= T)    x -= T;
  while (x < 0.0f)  x += T;
  return x;
}

static inline void axisLimits(int joint_idx, float& lo, float& hi) {
  int axis = joint_idx % 3;
  if (axis == 0)      { lo = YAW_LIMIT_LO_DEG;  hi = YAW_LIMIT_HI_DEG;  }
  else if (axis == 1) { lo = HIP_LIMIT_LO_DEG;  hi = HIP_LIMIT_HI_DEG;  }
  else                { lo = KNEE_LIMIT_LO_DEG; hi = KNEE_LIMIT_HI_DEG; }
}

const char* axisName(int joint_idx) {
  switch (joint_idx % 3) {
    case 0:  return "yaw ";
    case 1:  return "hip ";
    default: return "knee";
  }
}

#if defined(HAVE_EEPROM)
static inline int trimEepromAddr(int j) {
  return EE_TRIM_ADDR + j * (int)sizeof(float);
}
#endif

// Load mounted-horn trims.  On AVR (Mega) this reads the same EEPROM
// block the servo bridge writes; on other cores there's no EEPROM, so we
// keep zeros (untrimmed) -- "0 deg" is then the servo's electrical
// centre.
void loadTrims() {
  for (int i = 0; i < 18; ++i) trim_deg[i] = 0.0;
#if defined(HAVE_EEPROM)
  uint16_t magic = 0;
  EEPROM.get(EE_MAGIC_ADDR, magic);
  if (magic != EE_MAGIC) return;   // blank/older EEPROM -> leave zeros
  for (int i = 0; i < 18; ++i) {
    float v = 0.0;
    EEPROM.get(trimEepromAddr(i), v);
    if (v != v) v = 0.0;  // NaN guard
    trim_deg[i] = clampf(v, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
  }
#endif
}

// Persist the remembered standing foot Z.  AVR (Mega) only -- on the
// UNO Q (no <EEPROM.h>) this is a no-op and the value lives only in RAM
// for the session (see DEFAULT_STAND_Z to bake one in at compile time).
void saveStandZ() {
#if defined(HAVE_EEPROM)
  EEPROM.put(EE_STAND_MAGIC_ADDR, EE_STAND_MAGIC);
  EEPROM.put(EE_STAND_ADDR, g_stand_z);
#endif
}

// Reload a previously-remembered standing foot Z (AVR only).  Leaves the
// geometry/DEFAULT_STAND_Z value in place if none was ever saved.
void loadStandZ() {
#if defined(HAVE_EEPROM)
  uint16_t m = 0;
  EEPROM.get(EE_STAND_MAGIC_ADDR, m);
  if (m != EE_STAND_MAGIC) return;
  float v = g_stand_z;
  EEPROM.get(EE_STAND_ADDR, v);
  if (v == v) g_stand_z = v;   // NaN guard
#endif
}

// Drive one joint to an angle (clamped to its safe range, trim applied).
void writeJoint(int joint_idx, float angle_deg) {
  if (joint_idx < 0 || joint_idx >= 18) return;
  float lo, hi;
  axisLimits(joint_idx, lo, hi);
  angle_deg = clampf(angle_deg, lo, hi);
  current_deg[joint_idx] = angle_deg;

  float corrected = clampf(angle_deg + trim_deg[joint_idx], lo, hi);
  float us = PWM_MIN_US
           + (corrected + 90.0) / 180.0 * (PWM_MAX_US - PWM_MIN_US);

  // Routing (matches prototype_walk.ino): one board per 3 legs, so each
  // leg's 3 servos stay on ONE board.  legs 0..2 (joints 0..8) -> 0x40
  // ch 0..8; legs 3..5 (joints 9..17) -> 0x41 ch 0..8.
  int board = (joint_idx < 9) ? 0 : 1;
  if (!have_board[board]) return;   // absent driver: skip -> never wedge the bus
  Adafruit_PWMServoDriver& drv = (board == 0) ? pwm1 : pwm2;
  int chan = (joint_idx < 9) ? joint_idx : (joint_idx - 9);
  drv.writeMicroseconds(chan, (int)(us + 0.5));
}

// Wake a present PCA9685 from its power-on SLEEP state and set the 50 Hz
// servo rate.  Called once each time a board first appears (boot or `I`
// scan) -- defined here so SERVO_HZ is in scope.
void initBoard(int idx) {
  if (idx == 0) { pwm1.begin(); pwm1.setPWMFreq(SERVO_HZ); }
  else          { pwm2.begin(); pwm2.setPWMFreq(SERVO_HZ); }
}

void centreAll() {
  for (int i = 0; i < 18; ++i) writeJoint(i, 0.0);
}

// Turn every servo OFF: write the PCA9685 "full-off" bit on all channels
// so the PWM pulse stops and the servos de-energize (go limp, no holding
// torque).  Also drops out of auto/walk so nothing re-drives them.  Any
// later move/pose command re-energizes the relevant channels.
void relaxAll() {
  g_auto = false;
  g_walk = false;
  for (uint8_t ch = 0; ch < 16; ++ch) {
    if (have_board[0]) pwm1.setPWM(ch, 0, 4096);   // bit12 = full OFF -> no pulse
    if (have_board[1]) pwm2.setPWM(ch, 0, 4096);   // skip absent driver
  }
  say("OK motors OFF (relaxed/limp). Send any move/pose to re-energize.");
}

// --- Whole-body poses + tripod gait (ported from prototype_walk.ino) -----

// Blocking, raised-cosine eased move of all 18 joints from where they are
// now (current_deg[]) to `dest`, over dur_ms.  Used for the one-shot pose
// transitions so the servos start and stop gently instead of snapping.
void sweepAllTo(const float dest[18], unsigned long dur_ms) {
  float start[18];
  for (int i = 0; i < 18; ++i) start[i] = current_deg[i];
  unsigned long t0 = millis();
  for (;;) {
    unsigned long el = millis() - t0;
    float u = (dur_ms == 0) ? 1.0f : (float)el / (float)dur_ms;
    if (u > 1.0f) u = 1.0f;
    float s = 0.5f * (1.0f - cosf((float)PI * u));   // ease in / ease out
    for (int i = 0; i < 18; ++i)
      writeJoint(i, start[i] + (dest[i] - start[i]) * s);
    if (u >= 1.0f) break;
    delay(15);
  }
}

// Pose: raise every leg straight UP -- femurs to the up limit, knees
// straight, so the legs point up over the body ("legs over its head").
void poseLegsUp() {
  g_auto = false; g_walk = false;
  float dest[18];
  for (int i = 0; i < 6; ++i) {
    dest[i * 3 + 0] = 0.0f;
    dest[i * 3 + 1] = HIP_LIMIT_LO_DEG;   // hip pitch min => femur points up
    dest[i * 3 + 2] = 0.0f;               // knee straight => leg colinear, up
  }
  say("U -> raising legs UP over head (easing)...");
  sweepAllTo(dest, 1500);
  say("OK legs up");
}

// Pose: the walking stance -- feet planted under the body at the remembered
// standing height (g_stand_z), ready to walk.  Falls back to the nominal
// (0, -25, +60) angles if g_stand_z is somehow out of IK reach.
void poseStance() {
  g_auto = false; g_walk = false;
  float dest[18];
  if (!stanceDestAtZ(g_stand_z, dest)) {
    for (int i = 0; i < 6; ++i) {
      dest[i * 3 + 0] = 0.0f;
      dest[i * 3 + 1] = STANCE_HIP_DEG;
      dest[i * 3 + 2] = STANCE_KNEE_DEG;
    }
  }
  say(String("P -> easing into walking stance (foot Z ")
      + String(g_stand_z, 1) + " mm)...");
  sweepAllTo(dest, 1200);
  say("OK stance");
}

void initGaitConstants() {
  for (int i = 0; i < 6; ++i) {
    float a = (i + 0.5f) * (float)PI / 3.0f;   // leg azimuth
    leg_cos[i] = cosf(a);
    leg_sin[i] = sinf(a);
  }
  float p  = radians(STANCE_HIP_DEG);
  float pt = radians(STANCE_HIP_DEG + STANCE_KNEE_DEG);
  foot_neutral_x = COXA + FEMUR * cosf(p) + TIBIA * cosf(pt);
  foot_neutral_z = -FEMUR * sinf(p) - TIBIA * sinf(pt);
  foot_radius_eff = LEG_RADIAL + foot_neutral_x;
#ifdef DEFAULT_STAND_Z
  g_stand_z = (float)DEFAULT_STAND_Z;   // hardcoded (EEPROM-less boards)
#else
  g_stand_z = foot_neutral_z;           // default: the geometric stance height
#endif
}

// 2-link planar IK in the leg's yaw frame (port of _leg_ik).  Inputs mm;
// outputs hip pitch p and knee pitch k in RADIANS.  False = out of reach.
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

// Standing foot radial (yaw-frame x, mm) for a given foot Z: sprawled
// neutral at/above the neutral height, tucked in to STAND_TUCK_X_MM by
// STAND_TUCK_Z_MM, linear in between.  See the STAND_TUCK_* comment.
float stanceFootX(float foot_z) {
  if (foot_z >= foot_neutral_z) return foot_neutral_x;
  if (foot_z <= STAND_TUCK_Z_MM) return STAND_TUCK_X_MM;
  float t = (foot_z - foot_neutral_z) / (STAND_TUCK_Z_MM - foot_neutral_z);
  return foot_neutral_x + t * (STAND_TUCK_X_MM - foot_neutral_x);
}

// Fill an 18-joint destination for a symmetric standing pose with every
// foot at stanceFootX(foot_z) radial and the given foot Z (mm, more
// negative = lower feet / legs reaching further down).  Yaw stays 0 on
// all legs.  Returns false -- dest untouched -- if that Z is out of IK
// reach OR needs angles beyond the safe servo limits (otherwise
// writeJoint would clamp silently and the real pose would quietly stop
// matching the commanded one).
bool stanceDestAtZ(float foot_z, float dest[18]) {
  float p, k;
  if (!legIK(stanceFootX(foot_z), foot_z, p, k)) return false;
  float pd = degrees(p), kd = degrees(k);
  if (pd < HIP_LIMIT_LO_DEG || pd > HIP_LIMIT_HI_DEG) return false;
  if (kd < KNEE_LIMIT_LO_DEG || kd > KNEE_LIMIT_HI_DEG) return false;
  for (int i = 0; i < 6; ++i) {
    dest[i * 3 + 0] = 0.0f;
    dest[i * 3 + 1] = pd;
    dest[i * 3 + 2] = kd;
  }
  return true;
}

// One 50 Hz control update: advance the gait phase by dt seconds and drive
// all 18 joints (port of TripodGait, scalar scales = 1).
void stepGait(float dt) {
  g_elapsed += dt;
  g_phase = wrap2pi(g_phase + 2.0f * (float)PI * dt / fmaxf(g_period, 0.05f));
  float ramp_amp = fminf(g_elapsed / g_ramp, 1.0f);
  float t_eff = fmaxf(g_period, 0.05f);

  for (int i = 0; i < 6; ++i) {
    float ca = leg_cos[i], sa = leg_sin[i];
    int tripod = (i % 2 == 0) ? 0 : 1;
    float phi = wrap2pi(g_phase + PHASE_OFFSET + tripod * (float)PI);
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
    float fx_b = foot_radius_eff * ca + dx;
    float fy_b = foot_radius_eff * sa + dy;
    float rx = fx_b - LEG_RADIAL * ca;
    float ry = fy_b - LEG_RADIAL * sa;
    float x_yaw =  ca * rx + sa * ry;
    float y_yaw = -sa * rx + ca * ry;
    float yaw_angle = atan2f(y_yaw, x_yaw);
    float r_planar  = sqrtf(x_yaw * x_yaw + y_yaw * y_yaw);
    float p, k;
    if (legIK(r_planar, g_stand_z + dz, p, k)) {
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

// Ease into stance, then start the live tripod gait at vx mm/s forward
// (negative walks backward).  Speed is clamped to a sane bench range.
void startWalking(float vx) {
  g_auto = false;
  g_vx    = clampf(vx, -120.0f, 120.0f);
  g_vy    = 0.0f;
  g_omega = 0.0f;
  // Plant the gait's feet at the same (tucked) radial the standing pose
  // uses at the remembered height, so W walks from exactly where P stands.
  foot_radius_eff = LEG_RADIAL + stanceFootX(g_stand_z);
  poseStance();              // plant feet first so the gait eases in cleanly
  g_phase = 0.0f;
  g_elapsed = 0.0f;          // re-arm the ease-in ramp
  last_step_us = micros();
  g_walk = true;
  say(String("OK walking ") + String(g_vx, 0) + " mm/s -- any key or C to stop");
}

// `D` -- gradually lower (descend) all six feet, one small eased step at a
// time, until you say they have pressed into the ground, then REMEMBER that
// foot height (g_stand_z) so `P` (stance) and `W` (walk) plant at it.
//
// Each step drops the feet by step_mm (default DESCEND_STEP_MM = 0.5 mm),
// holding the neutral stance footprint, and prints the running foot Z so
// you can watch the real legs.  Press ANY key when the feet are firmly on
// the ground -- that key STOPS + remembers; `X` also relaxes, `C` re-centres.
// Pass a NEGATIVE step (e.g. `D -0.5`) to go the other way and RAISE the
// feet (body down) if you overshot.  Stops on its own at the IK reach limit.
void descendLegs(float step_mm) {
  g_auto = false; g_walk = false;
  if (step_mm == 0.0f) step_mm = DESCEND_STEP_MM;

  float dest[18];
  // Ease to the current standing height first so we begin from a known pose
  // (whatever the legs were doing before -- centred, up, mid-sweep).
  if (stanceDestAtZ(g_stand_z, dest)) sweepAllTo(dest, 800);

  say(String("D -> descending feet ") + String(step_mm, 2)
      + " mm/step from Z " + String(g_stand_z, 1)
      + " mm. Press ANY key when the feet press the ground (X=relax, C=centre).");

  // Drop any buffered newline so a leftover Enter doesn't instantly stop us.
  while (CONSOLE.available()) CONSOLE.read();

  float z = g_stand_z;
  char stopc = 0;
  for (;;) {
    float nz = z - step_mm;                 // more negative = feet move DOWN
    if (!stanceDestAtZ(nz, dest)) {
      say(String("   reached leg reach / servo limit at Z ") + String(z, 1)
          + " mm -- can't descend further.");
      break;
    }
    sweepAllTo(dest, DESCEND_SWEEP_MS);     // gentle eased step
    z = nz;
    say(String("   foot Z = ") + String(z, 1) + " mm  (feet tucked to radial "
        + String(stanceFootX(z), 0) + " mm)");

    if (CONSOLE.available()) { stopc = (char)CONSOLE.read(); break; }
    delay(DESCEND_HOLD_MS);
  }
  // Swallow the rest of the stop line so it isn't re-parsed as a command.
  while (CONSOLE.available()) CONSOLE.read();

  g_stand_z = z;
  saveStandZ();
  say(String("OK ground height remembered: foot Z = ") + String(g_stand_z, 1)
      + " mm (P and W now plant here)");
#if !defined(HAVE_EEPROM)
  say(String("   (UNO Q has no EEPROM -- kept for THIS session only. To make it")
      + " permanent, set  #define DEFAULT_STAND_Z " + String(g_stand_z, 1)
      + "  near the top and re-flash.)");
#endif

  if (stopc == 'X' || stopc == 'x') relaxAll();
  else if (stopc == 'C' || stopc == 'c') { centreAll(); say("OK C centred"); }
}

// Smoothly ramp one joint from its current angle to `to_deg` in ~1 deg
// steps so the servo eases over instead of slamming.
void rampTo(int joint_idx, float to_deg) {
  float from = current_deg[joint_idx];
  float lo, hi;
  axisLimits(joint_idx, lo, hi);
  to_deg = clampf(to_deg, lo, hi);
  float step = (to_deg >= from) ? 1.0 : -1.0;
  for (float a = from; (step > 0) ? (a < to_deg) : (a > to_deg); a += step) {
    writeJoint(joint_idx, a);
    delay(g_step_ms);
  }
  writeJoint(joint_idx, to_deg);
}

void announceJoint(int j) {
  float lo, hi;
  axisLimits(j, lo, hi);
  String s = ">> joint ";
  if (j < 10) s += ' ';
  s += j;
  s += "  leg ";   s += (j / 3);
  s += ' ';        s += axisName(j);
  s += "  board "; s += ((j < 9) ? "0x40" : "0x41");
  s += " ch ";     s += ((j < 9) ? j : j - 9);
  s += "  range "; s += (int)lo;
  s += "..";       s += (int)hi;
  s += " deg";
  say(s);
}

// Exercise one joint: 0 -> low side -> high side -> 0, scaled by amplitude.
void testJoint(int j) {
  if (j < 0 || j >= 18) { say("ERR joint"); return; }
  float lo, hi;
  axisLimits(j, lo, hi);
  float a_lo = lo * g_amplitude;
  float a_hi = hi * g_amplitude;

  announceJoint(j);
  rampTo(j, 0.0);     delay(200);
  rampTo(j, a_lo);    delay(350);   // toward the negative limit
  rampTo(j, a_hi);    delay(350);   // toward the positive limit
  rampTo(j, 0.0);     delay(200);
  say("   done (ENTER=next, R=repeat)");
}

// Quick "is it listening?" wiggle: nudge one joint a small amount around
// wherever it currently sits (NOT a full sweep), a couple of cycles, then
// return.  This is the gentle aliveness check -- just enough motion to see
// the servo respond without a big swing.  wiggle_deg defaults to 8 and is
// clamped to the joint's safe range.
void quickTestJoint(int j, float wiggle_deg) {
  if (j < 0 || j >= 18) { say("ERR joint"); return; }
  if (!have_board[(j < 9) ? 0 : 1]) {
    announceJoint(j);
    say("   (board absent -- nothing to drive)");
    return;
  }
  if (wiggle_deg <= 0.0f) wiggle_deg = 8.0f;
  float lo, hi;
  axisLimits(j, lo, hi);
  float c = current_deg[j];                 // wiggle around current position
  float a = clampf(c - wiggle_deg, lo, hi);
  float b = clampf(c + wiggle_deg, lo, hi);
  announceJoint(j);
  say(String("   quick wiggle +/-") + String(wiggle_deg, 0) + " deg -- watch for a twitch");
  for (int k = 0; k < 2; ++k) {             // two clear cycles
    rampTo(j, b);  delay(150);
    rampTo(j, a);  delay(150);
  }
  rampTo(j, c);                             // back to where it started
  say("   done");
}

void testLeg(int leg) {
  if (leg < 0 || leg > 5) { say("ERR leg"); return; }
  say("== leg " + String(leg) + " (yaw, hip, knee) ==");
  for (int axis = 0; axis < 3; ++axis) testJoint(leg * 3 + axis);
  g_joint = ((leg * 3 + 2) + 1) % 18;
}

void testAll() {
  say("== full tour: joints 0..17 ==");
  for (int j = 0; j < 18; ++j) testJoint(j);
  say("== tour complete ==");
  g_joint = 0;
}

// Does a device ACK at this 7-bit I2C address?
bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Read one 8-bit register from an I2C device.  ok=false on any NACK.
uint8_t i2cRead8(uint8_t addr, uint8_t reg, bool& ok) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) { ok = false; return 0; }
  if (Wire.requestFrom((int)addr, 1) != 1) { ok = false; return 0; }
  ok = true;
  return (uint8_t)Wire.read();
}

// Dump the key PCA9685 registers so we can prove the chip is actually
// configured (PRESCALE => the 50 Hz servo rate took) and that the last
// commanded pulse really landed (ch0 OFF count).  This is how we tell a
// real I2C-write path from a servo/mechanical fault.
void i2cDump(uint8_t addr, const char* who) {
  bool ok;
  uint8_t mode1 = i2cRead8(addr, 0x00, ok);          // MODE1
  if (!ok) { say(String("  0x") + String(addr, HEX) + " " + who + ": NO RESPONSE"); return; }
  uint8_t pre  = i2cRead8(addr, 0xFE, ok);           // PRESCALE
  uint8_t offL = i2cRead8(addr, 0x08, ok);           // LED0_OFF_L
  uint8_t offH = i2cRead8(addr, 0x09, ok);           // LED0_OFF_H
  int off = ((offH & 0x0F) << 8) | offL;
  String s = "  0x"; s += String(addr, HEX); s += ' '; s += who;
  s += ": MODE1=0x"; s += String(mode1, HEX);
  s += " PRESCALE="; s += pre; s += " (want ~121 for 50Hz)";
  s += " ch0_off="; s += off; s += " (want ~307 @ centre)";
  say(s);
}

void i2cRegTest() {
  say("PCA9685 register read-back:");
  i2cDump(0x40, "legs0-2");
  i2cDump(0x41, "legs3-5");
}

// Scan the I2C bus and report every device that ACKs, then call out the
// two PCA9685 servo-driver boards specifically.  This is how you confirm
// the MCU actually "sees" the servo drivers (no ACK => wiring/power/addr
// problem, and the servos can't move no matter what you command).
void i2cScan() {
  String s = "I2C scan:";
  int found = 0;
  for (uint8_t a = 1; a < 127; ++a) {
    if (i2cPresent(a)) {
      s += " 0x";
      if (a < 16) s += '0';
      s += String(a, HEX);
      ++found;
    }
  }
  if (found == 0) s += " (NOTHING found -- check SDA/SCL/GND wiring + 3V3)";
  say(s);
  // Refresh the present-board flags so the rest of the sketch only ever
  // drives boards that actually ACK (a fixed/replaced/late-connected board
  // is picked up here without a reflash).  The FIRST time a board appears
  // we initialise it (wake from SLEEP + 50 Hz) so a driver connected after
  // boot starts pulsing instead of sitting dead.  Drop the init flag when a
  // board vanishes so it re-inits cleanly if it comes back.
  for (int i = 0; i < 2; ++i) {
    uint8_t addr = (i == 0) ? 0x40 : 0x41;
    bool present = i2cPresent(addr);
    have_board[i] = present;
    if (present && !board_inited[i]) { initBoard(i); board_inited[i] = true; }
    if (!present) board_inited[i] = false;
  }
  say(String("  0x40 driver (legs 0-2): ") + (have_board[0] ? "OK" : "MISSING"));
  say(String("  0x41 driver (legs 3-5): ") + (have_board[1] ? "OK" : "MISSING"));
}

void printHelp() {
  String s = "OK prototype_servo_test\n";
  s += "  (idle at boot -- nothing moves until you send a command)\n";
  s += "  G         GO: start/resume the auto sweep (all joints)\n";
  s += "  <ENTER>   test NEXT joint\n";
  s += "  N <j>     test joint j (0..17), full sweep\n";
  s += "  Q <j> [d] quick wiggle joint j +/-d deg (default 8) -- is it listening?\n";
  s += "  L <leg>   test leg (0..5), all 3 joints\n";
  s += "  A         test ALL 18 in sequence\n";
  s += "  R         repeat last joint\n";
  s += "  C         centre all + stop\n";
  s += "  X         turn OFF all motors (relax/limp)\n";
  s += "  F <frac>  sweep amplitude 0.1..1.0\n";
  s += "  S         slow/fast sweep toggle\n";
  s += "  I         scan I2C bus (are the 0x40/0x41 drivers seen?)\n";
  s += "  -- whole-body --\n";
  s += "  U         legs UP over head\n";
  s += "  P         walking stance / position (plants at remembered height)\n";
  s += "  D [mm]    descend feet gradually + REMEMBER ground height (any key stops;\n";
  s += "            feet tuck inward as they drop so the legs can lift the body)\n";
  s += "  Z [mm]    show (or set) the remembered standing foot Z\n";
  s += "  W [mm/s]  walk forward (default 35; e.g. W 20). any key/C stops\n";
  s += "  ?         help";
  say(s);
}

void handleLine(char* line) {
  g_seen_input = true;   // silence the idle heartbeat once we hear from the user
  while (*line == ' ') ++line;
  char cmd = line[0];
  char* p = line + 1;

  // `G` (go) RESUMES the hands-free auto sweep; `W` starts walking; every
  // other command takes manual control by leaving both modes.  Stopping
  // the gait/sweep is just sending any other key (e.g. C to centre).
  if (cmd == 'G') { g_auto = true; g_walk = false; say("OK auto"); return; }
  if (cmd == 'U') { poseLegsUp();      return; }
  if (cmd == 'P') { poseStance();      return; }
  if (cmd == 'D') { descendLegs(atof(p)); return; }  // descend feet + remember height
  if (cmd == 'Z') {
    // `Z` prints the remembered standing foot Z; `Z <mm>` sets + saves it.
    while (*p == ' ') ++p;
    if (*p == '\0') {
      say(String("OK standing foot Z = ") + String(g_stand_z, 1) + " mm");
    } else {
      g_stand_z = atof(p);
      saveStandZ();
      say(String("OK standing foot Z set to ") + String(g_stand_z, 1)
          + " mm (P/W plant here; send P to apply)");
    }
    return;
  }
  if (cmd == 'W') {
    float vx = atof(p);              // `W` -> default slow; `W 20` -> 20 mm/s
    if (vx == 0.0f) vx = 35.0f;
    startWalking(vx);
    return;
  }
  g_auto = false;
  g_walk = false;

  // Bare ENTER -> test next joint and advance.
  if (cmd == '\0') {
    testJoint(g_joint);
    g_joint = (g_joint + 1) % 18;
    return;
  }
  if (cmd == '?' || cmd == 'H') { printHelp(); return; }
  if (cmd == 'I') { i2cScan(); return; }
  if (cmd == 'Y') { i2cRegTest(); return; }
  if (cmd == 'C') { centreAll(); say("OK C centred"); return; }
  if (cmd == 'X') { relaxAll(); return; }
  if (cmd == 'R') { testJoint(g_joint == 0 ? 17 : g_joint - 1); return; }
  if (cmd == 'A') { testAll(); return; }
  if (cmd == 'S') {
    g_step_ms = (g_step_ms == STEP_SLOW_MS) ? STEP_FAST_MS : STEP_SLOW_MS;
    say(String("OK speed ") + (g_step_ms == STEP_FAST_MS ? "fast" : "slow"));
    return;
  }
  if (cmd == 'F') {
    float f = atof(p);
    if (f < 0.1 || f > 1.0) { say("ERR frac 0.1..1.0"); return; }
    g_amplitude = f;
    say("OK amplitude " + String(g_amplitude, 2));
    return;
  }
  if (cmd == 'N') {
    int j = atoi(p);
    if (j < 0 || j >= 18) { say("ERR joint"); return; }
    testJoint(j);
    g_joint = (j + 1) % 18;
    return;
  }
  if (cmd == 'Q') {
    char* endp = p;
    long j = strtol(p, &endp, 10);     // joint
    float wig = atof(endp);            // optional wiggle deg (0 -> default 8)
    quickTestJoint((int)j, wig);
    g_joint = ((int)j + 1) % 18;
    return;
  }
  if (cmd == 'L') {
    testLeg(atoi(p));
    return;
  }
  say("ERR unknown (try ?)");
}

// Read any pending serial command lines (non-blocking).
void pumpSerial() {
  static char buf[64];
  static int  n = 0;
  static bool last_cr = false;   // for swallowing the LF of a CR+LF pair
  while (CONSOLE.available() > 0) {
    char c = (char)CONSOLE.read();
    // Accept EITHER CR or LF as end-of-line so the sketch works whether
    // the terminal sends \n, \r, or \r\n (raw-mode socat sends bare \r on
    // Enter).  A CR+LF pair is treated as ONE line ending so a bare Enter
    // still maps to a single "next joint" command.
    if (c == '\n' && last_cr) { last_cr = false; continue; }
    last_cr = (c == '\r');
    if (c == '\r' || c == '\n') {
      buf[n] = '\0';
      handleLine(buf);
      n = 0;
    } else if (n < (int)sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      n = 0;
      say("ERR line too long");
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  CONSOLE.begin(115200);
#if defined(CONSOLE_IS_BRIDGE)
  // Give the USB<->Linux bridge a moment to connect so the opening
  // banner isn't dropped before the Serial Monitor attaches (bounded so
  // it still runs headless if no monitor is ever opened).
  for (int i = 0; i < 30 && !CONSOLE; ++i) delay(100);
#endif
  Wire.begin();
  delay(50);
  loadTrims();    // match the bridge's mounted-horn calibration
  initGaitConstants();   // precompute leg azimuths + neutral foot pose (sets g_stand_z default)
  loadStandZ();   // override with a previously-remembered descend height (AVR EEPROM only)
  // i2cScan() probes the bus AND initialises every board it finds (wake +
  // 50 Hz).  Re-send `I` any time you connect a board after boot and it
  // will be configured -- no reflash, no power-cycle needed.
  i2cScan();
  centreAll();
  printHelp();

  // No motion until you ask for it: the sketch boots centred and idle.
  // Clamp the legs / put the feet in the air, then send `G` to begin the
  // (slow) auto sweep, or N/L/A/ENTER to test a single joint or leg.
  say("READY -- centred & idle. Send G to start the sweep (or N/L/A; ? = help).");
}

void loop() {
  pumpSerial();

  if (g_walk) {
    digitalWrite(LED_BUILTIN, HIGH);    // LED solid while walking
    unsigned long now = micros();
    unsigned long elapsed = now - last_step_us;   // wraps cleanly (unsigned)
    if (elapsed >= STEP_PERIOD_US) {
      last_step_us = now;
      stepGait((float)elapsed * 1e-6f);
    }
    return;
  }

  if (g_auto) {
    digitalWrite(LED_BUILTIN, HIGH);   // LED on while a joint is moving
    testJoint(g_joint);
    digitalWrite(LED_BUILTIN, LOW);
    g_joint = (g_joint + 1) % 18;
    // Pause between joints, but stay responsive to a stop command.
    for (int i = 0; i < g_auto_gap_ms / 20 && g_auto; ++i) {
      pumpSerial();
      delay(20);
    }
    return;
  }

  // Idle: booted, stopped, or waiting for the next command.  Slow-blink
  // the onboard LED so you can see the board is alive and waiting (and no
  // joint is being driven), then keep polling for a command.
  digitalWrite(LED_BUILTIN, (millis() / 600) & 1);
}
