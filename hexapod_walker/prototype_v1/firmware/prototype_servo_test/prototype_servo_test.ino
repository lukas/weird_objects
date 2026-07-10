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

      Physical wiring order on this robot is side-ordered, not circular:
        legs 0,1,2 = right side front -> rear
        legs 3,4,5 = left  side front -> rear
      The whole-body gait maps those wired legs back to CAD/sim azimuths
      internally, so `L 0` still tests the first physical right-side leg
      while `W` walks with the correct geometry.

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
      W [mm/s] [t|r|w]
                   walk forward (live gait) at the given speed; the optional
                   letter picks the gait: t = tripod (fast, default), r =
                   ripple (medium), w = wave (slowest, most stable).  e.g.
                   W 20 ,  W r ,  W 25 w .  Negative speed walks backward.
                   Any key / C stops.

      The three gaits are the same engine with different leg phasing + duty
      factor (fraction of the cycle each foot stays planted): tripod 0.55,
      ripple 2/3, wave 5/6.  Body speed is held at the commanded mm/s for
      every gait, so wave just takes more, smaller, overlapping steps -- use
      it if tripod walks too wobbly.

  --- Stability shaping (all gaits) ---

      Three things keep the body from rocking on top of the work the gaits do:
        * Double support -- tripod runs at duty 0.55 (not 0.5) so the two
          tripods overlap briefly; there's always >=3 feet down AND a short
          all-six window at each swap, removing the drop/pitch a zero-overlap
          swap causes.
        * COM weight-shift -- the body is nudged toward the centroid of the
          feet currently planted so the mass stays over the support polygon.
          This is small for the symmetric tripod (its support triangle is
          already centred) but real when turning or in ripple/wave.  Use
          `E <x> <y>` to add a STATIC lean (mm) that cancels a known load
          imbalance (e.g. the wire bundle tugging the robot to one side) --
          that static trim is usually the single biggest COM win here.
        * Cycloidal, velocity-matched swing -- the swing foot's horizontal
          motion is a Hermite whose end velocity equals the stance speed, so
          the foot is already moving with the ground (zero scrub) at both
          lift-off and touch-down.  Stance keeps the foot planted at constant
          ground speed.  `K <mm>` lowers the swing lift for an even steadier,
          flatter step.

      Other stability levers live in the geometry: `D` / `Z` set the standing
      foot height (lower body + the feet tuck inward = lower COM, bigger tilt
      margin).  Physically: fit the TPU spike boots (traction), route the wire
      harness low/centred, and a shorter tibia would cut servo sag + COM
      height further.

  --- Dances (live 50 Hz moves; any key stops) ---

      V            stadium wave: a crest of raised femur+tibia travels
                   around the body azimuth like a sports-crowd wave
      O            say hi: weight shifts onto the rear four legs, the two
                   FRONT legs lift up and wave side-to-side
      B            hula: feet stay planted, the body sways in a circle
      T            twist & dip: the body twists on its yaws while bouncing

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
// Remembered stand-up RISE tuck-in radius (the `$` command); same AVR-only
// persistence as the stand Z above (UNO Q keeps it in RAM for the session).
const uint16_t EE_STAND_TUCK_MAGIC      = 0x5452;  // 'T','R' (tuck radius)
const int      EE_STAND_TUCK_MAGIC_ADDR = EE_STAND_ADDR + (int)sizeof(float);  // 82
const int      EE_STAND_TUCK_ADDR       = EE_STAND_TUCK_MAGIC_ADDR + (int)sizeof(uint16_t);  // 84
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

// --- Staged STAND-UP sequence (peak-torque / peak-current reduction) --------
// Standing from a wide sprawl by shoving all six legs up at once is what
// browned out / smoked the rail: (1) sprawled feet ~200 mm out out-lever the
// femur/tibia servos, so the lift is near-stall, and (2) all 18 servos hit
// that near-stall load in the SAME instant, summing their currents into one
// spike.  The `P` (Stand) command therefore no longer eases straight to the
// final pose -- it runs standUp(), which:
//   1. FOLDS to the lowest safe crouch (body resting, feet barely below the
//      hips) so the rise begins from a known, lightly-loaded pose;
//   2. RISES to the target height through several slew-limited waypoints,
//      lifting ONE tripod (3 legs / 9 servos) at a time so the moving-servo
//      current is ~halved vs an all-six lift, while >=3 feet stay planted at
//      every instant (statically stable).  The feet tuck INWARD as the body
//      rises (shorter moment arm = less torque exactly where the lift is
//      hardest): at each height the feet are pulled in as TIGHT as the knee/hip
//      limits allow (aiming for STAND_RISE_RADIUS_MM), relaxing outward only as
//      far as needed -- so the tuck tightens smoothly instead of snapping in
//      from a full sprawl at the crouch heights where a tight tuck can't reach;
//   3. SPREADS the feet out to the final stance footprint at the target
//      height, landing in exactly the pose poseStance()/the gait expect.
// Everything stays inside the existing IK + servo angle limits and is gated
// behind ARM.  The FINAL stance radius is governed by the existing STANCE /
// STAND_TUCK_* geometry (stanceFootX(g_stand_z)) so `P`, `W` and the dances
// all launch cleanly from the result -- it is intentionally not overridden
// here.  Tune the rise itself with:
#define STAND_RISE_RADIUS_MM 130.0f  // DEFAULT rise tuck-in radial; live via `$`
#define STAND_RISE_STEPS     6       // slew-limited height waypoints fold -> target
#define STAND_STEP_MS        170     // ease time per tripod sub-move (slew-rate limit)
#define STAND_FOLD_MS        900     // ease time into the initial folded crouch
#define STAND_SETTLE_MS      450     // final eased spread onto the stance footprint

// --- Pre-lift CROUCH / TUCK (the `CROUCH` command) --------------------------
// The FOLD/crouch stage of the stand-up exposed on its OWN so the feet can be
// pulled in under the body -- gradually, statically stable -- WHILE THE BODY
// STAYS LOW, without lifting into a stand.  Watch it settle, then send `P`
// (Stand) to lift.  Same slew-limited, tripod-staggered, progressive-relax
// approach the rise uses (tuckDestAt) so the knee never jams at its limit.
#define CROUCH_FOLD_MS    900     // ease down into the low sprawled crouch first
#define CROUCH_TUCK_STEPS 5       // slew-limited waypoints tightening the tuck
#define CROUCH_STEP_MS    170     // ease time per tripod sub-move (slew-rate limit)

// --- Graceful SIT-DOWN / LOWER (the `SIT` / `SETTLE` commands) ---------------
// The INVERSE of standUp(): from the current standing pose, step the body
// height DOWN over slew-limited waypoints -- lifting ONE tripod at a time so
// >=3 feet always support the body -- to a low resting pose, then spread onto a
// wide base.  Only THEN is it safe to relax/limp, so the robot settles gently
// instead of smashing to the ground.  A touch slower than the rise (gravity
// helps) so each step stays controlled.
#define SIT_LOWER_STEPS 6         // slew-limited height waypoints stand -> rest
#define SIT_STEP_MS     190       // ease time per tripod sub-move (slew-rate limit)
#define SIT_SETTLE_MS   450       // final eased spread onto a wide resting base

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
// SAFETY GATE: the robot boots DISARMED -- every PCA9685 channel is forced to
// "full OFF" (no PWM pulse) so all 18 servos sit LIMP (no holding torque, no
// stall current) until a human explicitly sends `ARM` from the web page.  When
// disarmed, ALL servo-driving commands are refused as a backstop.  There is no
// software cutoff of the servo V+ rail (BEC power via the manual anti-spark
// switch), so DISARMED == "powered but receiving no signal", the software
// equivalent of unpowered.  `X` / `DISARM` returns to this state (e-stop).
bool  g_armed      = false;
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

// --- Dance modes (V/O/B/T commands) -- live 50 Hz moves like the walk ----
// 0 = off, 1 = V stadium wave, 2 = O hi-wave, 3 = B hula, 4 = T twist&dip.
int   g_dance = 0;
float g_dance_phase   = 0.0f;
float g_dance_elapsed = 0.0f;

// Leg geometry (mm) + stance angles -- mirror hexapod_prototype.py.
const float COXA  =  25.0;
const float FEMUR =  75.0;   // Jun 2026: shortened from 90 (coxa-link crack fix)
const float TIBIA = 130.0;
const float LEG_RADIAL      = 100.0;   // chassis apothem (flat-to-flat / 2)
const float STANCE_HIP_DEG  = -25.0;
const float STANCE_KNEE_DEG =  60.0;

// Gait parameters -- slow, gentle defaults for bench bring-up.
float g_period = 1.00;     // s  -- full gait cycle (slow cadence)
float g_lift   = 18.0;     // mm -- peak swing foot lift (lower = steadier)
float g_ramp   = 0.45;     // s  -- ease-in for stride + lift
float g_vx     = 35.0;     // mm/s forward (slow); chassis +X
float g_vy     = 0.0;      // mm/s lateral
float g_omega  = 0.0;      // rad/s yaw

// --- Stability shaping ----------------------------------------------------
// Body weight-shift: each tick the body is nudged toward the centroid of the
// feet currently PLANTED, so the centre of mass stays over the loaded
// support polygon.  g_com_gain scales the live (gait-driven) part; g_com_x/
// g_com_y are a STATIC trim (chassis +X forward, +Y left, mm) to cancel a
// known load imbalance -- e.g. set these if the wire bundle/battery makes the
// robot lean.  Tunable live with `E <x> <y>`.  See the header "stability"
// note for why this is small for a symmetric tripod but matters when turning
// or in ripple/wave.
float g_com_gain = 0.5f;   // 0 = off; how hard to track the support centroid
float g_com_x    = 0.0f;   // static fore/aft COM trim (mm, + = lean forward)
float g_com_y    = 0.0f;   // static lateral COM trim (mm, + = lean left)

// --- Selectable hexapod gait ---------------------------------------------
// All three real insect gaits are the SAME engine with different per-leg
// phase offsets + duty factor (= fraction of the cycle a foot is planted):
//   0 TRIPOD  beta 1/2 : two alternating sets of 3 -> fast, least stable
//             (this is what the robot walked in the wobbly first video)
//   1 RIPPLE  beta 2/3 : a diagonal wave, ~2 feet swinging -> medium
//   2 WAVE    beta 5/6 : one foot at a time, back->front -> slow, rock-steady
// leg_phase[] is keyed by WIRED leg index (0..2 right front->rear, 3..5
// left front->rear).  Body speed is held at g_vx for every gait by scaling
// stride with the stance time (beta*period), so a more stable gait just
// takes more, smaller, overlapping steps -- it doesn't crawl slower for the
// same command (though its default speed is gentler).
int   g_gait  = 0;         // 0 tripod, 1 ripple, 2 wave, 3 tetrapod
float g_duty  = 0.5f;      // beta: fraction of the cycle each foot is planted
float leg_phase[6] = {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f};   // tripod default

// Precomputed per-leg constants (filled by initGaitConstants()).
float leg_cos[6], leg_sin[6];
float foot_neutral_x, foot_neutral_z, foot_radius_eff;
// Standing foot Z actually used by the stance pose + gait baseline.  Set
// from geometry (or DEFAULT_STAND_Z / EEPROM) at boot, then refined live by
// the `D` descend command and remembered.
float g_stand_z = 0.0f;
// Stand-up RISE tuck-in radius (yaw-frame foot radial, mm): how tight standUp()
// pulls the feet in under the body during the staggered rise -- smaller = tucked
// tighter = shorter moment arm = more lift leverage (watch the knee limit).  This
// is only the RISE tuck; the FINAL stance radius stays governed by stanceFootX().
// Defaults to STAND_RISE_RADIUS_MM; set live with `$ <mm>` (AVR-persisted like Z).
float g_stand_tuck_r = STAND_RISE_RADIUS_MM;

// Physical wiring order: 0,1,2 down the RIGHT side (front->rear), then
// 3,4,5 down the LEFT side (front->rear).  Convert each wired leg index
// to the CAD/sim circular leg index (azimuth = (cad + 0.5) * 60 deg).
const int WIRED_TO_CAD_LEG[6] = {5, 4, 3, 0, 1, 2};

// Forward declarations (definitions appear further down).
bool legIK(float target_x, float target_z, float& p, float& k);
float stanceFootX(float foot_z);
bool stanceDestAtZ(float foot_z, float dest[18]);
bool poseDestAt(float foot_x, float foot_z, float dest[18]);
bool tuckDestAt(float foot_z, float aim_x, float dest[18]);
bool riseDestAt(float foot_z, float dest[18]);
void descendLegs(float step_mm);
void setGait(int g);
void standUp();
void crouchDown();
void sitDown();

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

// Persist / reload the stand-up rise tuck-in radius (AVR only) -- same pattern
// as saveStandZ/loadStandZ.  On the UNO Q (no EEPROM) these are no-ops and the
// value lives only for the session (default STAND_RISE_RADIUS_MM at boot).
void saveStandTuck() {
#if defined(HAVE_EEPROM)
  EEPROM.put(EE_STAND_TUCK_MAGIC_ADDR, EE_STAND_TUCK_MAGIC);
  EEPROM.put(EE_STAND_TUCK_ADDR, g_stand_tuck_r);
#endif
}

void loadStandTuck() {
#if defined(HAVE_EEPROM)
  uint16_t m = 0;
  EEPROM.get(EE_STAND_TUCK_MAGIC_ADDR, m);
  if (m != EE_STAND_TUCK_MAGIC) return;
  float v = g_stand_tuck_r;
  EEPROM.get(EE_STAND_TUCK_ADDR, v);
  if (v == v) g_stand_tuck_r = v;   // NaN guard
#endif
}

// Per-board PCA channel for a joint.  Default rule: ch = joint % 9 on the
// leg's board.  Hardware exceptions are remapped here -- Jun 2026: channel
// 6 on the 0x41 board is BROKEN, so leg 5 yaw (joint 15) is physically
// plugged into 0x41 ch 9 instead.
static inline int jointChannel(int joint_idx) {
  if (joint_idx == 15) return 9;   // leg 5 yaw: 0x41 ch6 dead -> moved to ch9
  return (joint_idx < 9) ? joint_idx : (joint_idx - 9);
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
  drv.writeMicroseconds(jointChannel(joint_idx), (int)(us + 0.5));
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

// Write the PCA9685 "full-off" bit on every channel of each present board so
// the PWM pulse stops and the servos de-energize (go limp, no holding torque).
// This is the raw output-kill used by both boot and DISARM.  A later real
// writeMicroseconds() on a channel clears its full-off bit and re-energizes it.
void disarmOutputs() {
  for (uint8_t ch = 0; ch < 16; ++ch) {
    if (have_board[0]) pwm1.setPWM(ch, 0, 4096);   // bit12 = full OFF -> no pulse
    if (have_board[1]) pwm2.setPWM(ch, 0, 4096);   // skip absent driver
  }
}

// DISARM = the safe / emergency-stop state: cut all PWM (servos limp) AND set
// the armed gate false so NOTHING can re-drive them until the human ARMs
// again.  Also drops out of auto/walk/dance so no live mode re-energizes a
// channel.  This is what the robot boots into and what `X` / `DISARM` do.
void relaxAll() {
  g_auto  = false;
  g_walk  = false;
  g_dance = 0;
  g_armed = false;
  disarmOutputs();
  say("OK DISARMED -- all servos OFF/limp (no PWM). Send ARM to enable outputs.");
}

// ARM = enable outputs, but WITHOUT moving anything: we deliberately write no
// angles here, so every channel stays in its full-off (no-pulse) state and the
// servos remain limp until an explicit motion command (e.g. `P` stand) re-
// energizes them gradually.  This guarantees arming never slams a joint or
// spikes stall current.  The human presses Stand/Park afterwards to stand.
void arm() {
  g_armed = true;
  // Make sure any present board is awake / configured (normally already done
  // at boot; re-assert in case a driver was hot-plugged after boot).
  for (int i = 0; i < 2; ++i)
    if (have_board[i] && !board_inited[i]) { initBoard(i); board_inited[i] = true; }
  say("OK ARMED -- outputs enabled; joints stay LIMP until you move/Stand (send P).");
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

// Staged, low-peak-current stand-up -- what the `P` (Stand) command now runs.
// See the big STAND_* comment near the top for WHY (fold -> tripod-staggered
// rise -> spread), which lowers both peak servo torque and peak simultaneous
// current so the robot can stand from a sprawl without stalling/browning out.
// Ends in exactly the pose poseStance() would, so walking/dances launch cleanly.
void standUp() {
  g_auto = false; g_walk = false; g_dance = 0;
  float dest[18], full[18];

  // Stage 1 -- FOLD: ease every leg to the lowest safe stance (body resting,
  // feet just below the hips).  A known, lightly-loaded starting pose no matter
  // where the legs were (sprawled, centred, mid-sweep).  fold_z is the SHALLOW
  // end of the rise; never start deeper than the target itself.
  float fold_z = fmaxf(foot_neutral_z, g_stand_z);
  say(String("P -> stand: folding to a low crouch (foot Z ")
      + String(fold_z, 1) + " mm)...");
  if (stanceDestAtZ(fold_z, dest)) sweepAllTo(dest, STAND_FOLD_MS);

  // Stage 2 -- RISE: step body height from fold_z down to g_stand_z over
  // STAND_RISE_STEPS slew-limited waypoints, lifting ONE tripod at a time.
  // Only three legs (nine servos) ever move into the load at once, so peak
  // current is ~halved vs an all-six lift.  At each waypoint riseDestAt() tucks
  // the feet in as TIGHT as the knee/hip limits allow (down to g_stand_tuck_r),
  // relaxing outward only as far as needed -- so the moment arm stays as short
  // as possible for max lift leverage at EVERY height, and the tuck tightens
  // smoothly step to step instead of snapping in from a full sprawl in one
  // loaded move (which jammed the rise at the crouch heights).
  for (int s = 1; s <= STAND_RISE_STEPS; ++s) {
    float z = fold_z + (g_stand_z - fold_z) * ((float)s / (float)STAND_RISE_STEPS);
    if (!riseDestAt(z, full)) continue;              // unreachable waypoint: skip
    for (int grp = 0; grp < 2; ++grp) {              // two interleaved tripods
      for (int i = 0; i < 18; ++i) dest[i] = current_deg[i];   // hold the others
      for (int i = 0; i < 6; ++i) {
        if ((WIRED_TO_CAD_LEG[i] & 1) != grp) continue;        // CAD 0,2,4 then 1,3,5
        dest[i * 3 + 0] = full[i * 3 + 0];
        dest[i * 3 + 1] = full[i * 3 + 1];
        dest[i * 3 + 2] = full[i * 3 + 2];
      }
      sweepAllTo(dest, STAND_STEP_MS);
    }
  }

  // Stage 3 -- SPREAD: ease the feet out to the final stance footprint at the
  // target height (a near-constant-height slide -> little vertical load),
  // landing exactly where poseStance()/the gait expect.
  say(String("   settling to stance footprint (foot Z ")
      + String(g_stand_z, 1) + " mm)...");
  if (stanceDestAtZ(g_stand_z, dest)) sweepAllTo(dest, STAND_SETTLE_MS);
  else poseStance();
  say("OK standing");
}

// Pre-lift CROUCH / TUCK -- the FOLD stage of standUp() on its own.  Ease the
// body down to the low crouch height, then pull the feet INWARD toward the tuck
// radius (g_stand_tuck_r) GRADUALLY over CROUCH_TUCK_STEPS slew-limited
// waypoints, moving ONE tripod (3 legs) at a time so >=3 feet always stay
// planted (statically stable).  It does NOT lift into a stand: it leaves the
// robot low and tucked so the human can watch it settle, then press `P` (Stand)
// to lift from there.  tuckDestAt() relaxes the tuck outward only as far as the
// knee/hip limits allow, so the knee never jams at its limit.
void crouchDown() {
  g_auto = false; g_walk = false; g_dance = 0;
  float dest[18], full[18];
  float crouch_z = fmaxf(foot_neutral_z, g_stand_z);   // low, resting body height

  // Ease to the sprawled stance at the crouch height first (a known, lightly
  // loaded starting pose no matter where the legs were).
  say(String("CROUCH -> easing to a low crouch (foot Z ")
      + String(crouch_z, 1) + " mm), then tucking the feet in...");
  if (stanceDestAtZ(crouch_z, dest)) sweepAllTo(dest, CROUCH_FOLD_MS);

  // Tighten the tuck from the full sprawl toward g_stand_tuck_r a few mm per
  // waypoint, one tripod at a time, so the feet pull in smoothly instead of
  // snapping in under load.
  float x_wide = stanceFootX(crouch_z);
  float x_aim  = fminf(g_stand_tuck_r, x_wide);
  for (int s = 1; s <= CROUCH_TUCK_STEPS; ++s) {
    float aim = x_wide + (x_aim - x_wide) * ((float)s / (float)CROUCH_TUCK_STEPS);
    if (!tuckDestAt(crouch_z, aim, full)) continue;    // unreachable tuck: skip
    for (int grp = 0; grp < 2; ++grp) {                // two interleaved tripods
      for (int i = 0; i < 18; ++i) dest[i] = current_deg[i];   // hold the others
      for (int i = 0; i < 6; ++i) {
        if ((WIRED_TO_CAD_LEG[i] & 1) != grp) continue;
        dest[i * 3 + 0] = full[i * 3 + 0];
        dest[i * 3 + 1] = full[i * 3 + 1];
        dest[i * 3 + 2] = full[i * 3 + 2];
      }
      sweepAllTo(dest, CROUCH_STEP_MS);
    }
  }
  say("OK crouched -- feet tucked low. Send P (Stand) to lift.");
}

// Graceful SIT-DOWN / LOWER -- the INVERSE of standUp().  From the current
// standing pose, step the body height DOWN from g_stand_z to the low rest
// height over SIT_LOWER_STEPS slew-limited waypoints, moving ONE tripod at a
// time so >=3 feet always support the body (statically stable) and it descends
// in small controlled steps instead of dropping.  riseDestAt() keeps the feet
// tucked tight at each height (short moment arm = the servos hold the descent
// easily).  Finishes by spreading the feet onto a WIDE base at the low height,
// so afterwards it is safe to relax/limp without the robot toppling.  Does NOT
// cut PWM itself -- the caller (SIT holds the low pose; SETTLE then DISARMs).
void sitDown() {
  g_auto = false; g_walk = false; g_dance = 0;
  float dest[18], full[18];
  float rest_z = fmaxf(foot_neutral_z, g_stand_z);   // low resting body height

  say(String("SIT -> lowering from stand (foot Z ") + String(g_stand_z, 1)
      + " mm) to a low rest (foot Z " + String(rest_z, 1) + " mm)...");
  for (int s = 1; s <= SIT_LOWER_STEPS; ++s) {
    float z = g_stand_z + (rest_z - g_stand_z) * ((float)s / (float)SIT_LOWER_STEPS);
    if (!riseDestAt(z, full)) continue;                // unreachable waypoint: skip
    for (int grp = 0; grp < 2; ++grp) {                // two interleaved tripods
      for (int i = 0; i < 18; ++i) dest[i] = current_deg[i];   // hold the others
      for (int i = 0; i < 6; ++i) {
        if ((WIRED_TO_CAD_LEG[i] & 1) != grp) continue;
        dest[i * 3 + 0] = full[i * 3 + 0];
        dest[i * 3 + 1] = full[i * 3 + 1];
        dest[i * 3 + 2] = full[i * 3 + 2];
      }
      sweepAllTo(dest, SIT_STEP_MS);
    }
  }

  // Spread the feet out to the sprawled stance at the low rest height so the
  // body settles on a WIDE, stable base -- now it is safe to relax/limp.
  say("   settling onto a wide resting base...");
  if (stanceDestAtZ(rest_z, dest)) sweepAllTo(dest, SIT_SETTLE_MS);
  say("OK seated -- low & stable. Safe to relax (DISARM) now.");
}

void initGaitConstants() {
  for (int i = 0; i < 6; ++i) {
    float a = (WIRED_TO_CAD_LEG[i] + 0.5f) * (float)PI / 3.0f;
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

// Like stanceDestAtZ but at an EXPLICIT foot radial (mm, yaw frame) instead of
// the geometry-derived stanceFootX -- used by the staged stand-up to hold the
// feet tucked in (short moment arm) during the rise.  Yaw stays 0 on all legs.
// Returns false (dest untouched) if that (x, z) is out of IK reach OR needs a
// hip/knee angle beyond the safe servo limits, so the caller can fall back to a
// sprawled-but-reachable stance pose at heights where this tuck is too tight.
bool poseDestAt(float foot_x, float foot_z, float dest[18]) {
  float p, k;
  if (!legIK(foot_x, foot_z, p, k)) return false;
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

// Tightest reachable tucked pose at a given foot Z for a desired tuck-in radial
// `aim_x`.  Start at aim_x and relax OUTWARD only as far as the knee/hip limits +
// IK reach require, never tighter than that and never wider than the sprawled
// stance radius stanceFootX(foot_z).  This keeps the foot moment arm as SHORT as
// the joints allow at EVERY height (max lift leverage) and lets the tuck tighten
// a few mm per waypoint as the body rises/lowers, instead of the old all-or-
// nothing "full tuck or full sprawl" that snapped the feet in ~50-70 mm in a
// single loaded sub-move (the jam that stalled the rise at crouch heights).
// Falls back to the full sprawled stance only if nothing tighter fits.  Yaw 0.
// Shared by the staged rise (standUp), the pre-lift crouch (crouchDown) and the
// graceful lower (sitDown).
bool tuckDestAt(float foot_z, float aim_x, float dest[18]) {
  float x_wide = stanceFootX(foot_z);              // sprawled-but-safe radius
  float x_tuck = fminf(aim_x, x_wide);             // never tuck wider than sprawl
  for (float x = x_tuck; x < x_wide; x += 2.0f)
    if (poseDestAt(x, foot_z, dest)) return true;  // tightest tuck that fits
  return stanceDestAtZ(foot_z, dest);              // last resort: full sprawl
}

// Tightest reachable stand-up RISE pose at a given foot Z -- the tuck aimed at
// the live rise tuck-in radius g_stand_tuck_r (settable via `$`).
bool riseDestAt(float foot_z, float dest[18]) {
  return tuckDestAt(foot_z, g_stand_tuck_r, dest);
}

// Select a gait: load its per-leg phase offsets, duty factor (beta) and a
// sensible cadence.  Phase tables are keyed by WIRED leg index
// (0..2 = right front->rear, 3..5 = left front->rear).
void setGait(int g) {
  // TRIPOD: {rf,rr,lm} swing together, {rm,lf,lr} half a cycle later.
  static const float TRIPOD[6] = {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f};
  // RIPPLE: diagonal wave alternating sides front->rear
  //   lf, rm, lr, rf, lm, rr at 0, 1/6, 2/6, 3/6, 4/6, 5/6  (by wired idx):
  static const float RIPPLE[6] = {3.0f/6, 1.0f/6, 5.0f/6, 0.0f/6, 4.0f/6, 2.0f/6};
  // WAVE: one foot at a time, right side back->front then left back->front
  //   rr, rm, rf, lr, lm, lf at 0, 1/6, ... 5/6  (by wired idx):
  static const float WAVE[6]   = {2.0f/6, 1.0f/6, 0.0f/6, 5.0f/6, 4.0f/6, 3.0f/6};
  // TETRAPOD (a.k.a. 4+2): three pairs of OPPOSITE legs swing in turn, so 4
  // feet are always down.  More stable than tripod, quicker than ripple/wave.
  //   pairs (CAD 0&3, 1&4, 2&5) at phase 0, 1/3, 2/3  -> by wired idx:
  static const float TETRA[6]  = {4.0f/6, 2.0f/6, 0.0f, 0.0f, 2.0f/6, 4.0f/6};
  const float* src;
  // Duty (beta) = fraction of the cycle each foot is planted.  Tripod uses
  // 0.55 (not 0.5) so the two tripods OVERLAP briefly -> short windows of
  // all-six ground contact at each swap, which kills the pitch/drop that a
  // zero-overlap swap causes.  Ripple/wave/tetrapod overlap inherently.
  if (g == 3)      { src = TETRA;  g_duty = 2.0f / 3.0f; g_period = 1.05f; }
  else if (g == 2) { src = WAVE;   g_duty = 5.0f / 6.0f; g_period = 1.50f; }
  else if (g == 1) { src = RIPPLE; g_duty = 2.0f / 3.0f; g_period = 1.10f; }
  else             { src = TRIPOD; g_duty = 0.55f;       g_period = 1.00f; g = 0; }
  for (int i = 0; i < 6; ++i) leg_phase[i] = src[i];
  g_gait = g;
}

const char* gaitName(int g) {
  return (g == 3) ? "tetrapod" : (g == 2) ? "wave" : (g == 1) ? "ripple" : "tripod";
}

// One 50 Hz control update: advance the gait phase by dt seconds and drive
// all 18 joints.  Generalised tripod/ripple/wave gait -- each leg follows
// the same swing/stance profile, offset by leg_phase[i] (fraction of cycle)
// with a stance (planted) fraction of g_duty.  Body speed is held at g_vx
// by scaling stride with the stance time (g_duty * period).
//
// Two stability features layered on the base gait:
//  * COM weight-shift -- pass 1 finds the centroid of the planted feet and
//    nudges the body toward it (+ static g_com trim) so the mass stays over
//    the support polygon.
//  * Cycloidal, velocity-matched swing -- the swing foot's horizontal motion
//    is a Hermite whose end-velocity equals the stance speed, so the foot is
//    moving with the ground (zero scrub) at both lift-off and touch-down.
void stepGait(float dt) {
  g_elapsed += dt;
  g_phase = wrap2pi(g_phase + 2.0f * (float)PI * dt / fmaxf(g_period, 0.05f));
  float ramp_amp = fminf(g_elapsed / g_ramp, 1.0f);
  float t_eff = fmaxf(g_period, 0.05f);
  float swing = 1.0f - g_duty;                 // fraction of cycle in the air
  float base_u = g_phase / (2.0f * (float)PI); // global cycle position 0..1
  // Swing end-tangent (in tau units) so the foot leaves/meets the ground at
  // the stance speed (-1/g_duty per unit u) -> no scrub at lift-off/touch-down.
  float Mtan = (swing > 1e-4f) ? (-swing / g_duty) : 0.0f;

  // Lift gate: fade the swing lift out as the commanded speed -> 0 so a
  // centred joystick just STANDS (feet planted, no marching in place) yet
  // resumes stepping instantly when pushed.  Below ~8 mm/s the feet stay down.
  float speed_mag = sqrtf(g_vx * g_vx + g_vy * g_vy)
                  + fabsf(g_omega) * foot_radius_eff;
  float lift_gate = clampf(speed_mag / 8.0f, 0.0f, 1.0f);

  // --- Pass 1: body weight-shift toward the planted-foot centroid ----------
  float sumx = 0.0f, sumy = 0.0f;
  int   nstance = 0;
  for (int i = 0; i < 6; ++i) {
    float u = base_u + leg_phase[i];
    while (u >= 1.0f) u -= 1.0f;
    if (u < swing) continue;                   // airborne -> not a support foot
    float ca = leg_cos[i], sa = leg_sin[i];
    float prog = 0.5f - (u - swing) / g_duty;
    float vx_at = g_vx - g_omega * foot_radius_eff * sa;
    float vy_at = g_vy + g_omega * foot_radius_eff * ca;
    sumx += foot_radius_eff * ca + prog * vx_at * g_duty * t_eff * ramp_amp;
    sumy += foot_radius_eff * sa + prog * vy_at * g_duty * t_eff * ramp_amp;
    ++nstance;
  }
  float ox = g_com_x, oy = g_com_y;            // static trim
  if (nstance > 0) {                           // + live tracking of support
    ox += g_com_gain * sumx / (float)nstance;
    oy += g_com_gain * sumy / (float)nstance;
  }
  ox *= ramp_amp;  oy *= ramp_amp;             // ease the shift in with the gait

  // --- Pass 2: place every foot with the body displaced by (ox, oy) --------
  for (int i = 0; i < 6; ++i) {
    float ca = leg_cos[i], sa = leg_sin[i];
    float u = base_u + leg_phase[i];           // this leg's cycle position
    while (u >= 1.0f) u -= 1.0f;               // frac into [0,1)
    float prog, dz;
    if (u < swing) {                           // SWING: cycloidal, vel-matched
      float tau = u / swing;
      float t2 = tau * tau, t3 = t2 * tau;
      float h00 =  2.0f * t3 - 3.0f * t2 + 1.0f;   // Hermite basis
      float h10 =         t3 - 2.0f * t2 + tau;
      float h01 = -2.0f * t3 + 3.0f * t2;
      float h11 =         t3 -        t2;
      prog = h00 * (-0.5f) + h01 * (0.5f) + (h10 + h11) * Mtan;
      dz = g_lift * ramp_amp * lift_gate * sinf((float)PI * tau);
    } else {                                   // STANCE: planted, push back
      float s = (u - swing) / g_duty;
      prog = 0.5f - s;
      dz = 0.0f;
    }
    float v_x_at = g_vx - g_omega * foot_radius_eff * sa;
    float v_y_at = g_vy + g_omega * foot_radius_eff * ca;
    // stride = body speed * stance time -> body advances at v_x during stance
    float dx = prog * v_x_at * g_duty * t_eff * ramp_amp;
    float dy = prog * v_y_at * g_duty * t_eff * ramp_amp;
    float fx_b = foot_radius_eff * ca + dx - ox;
    float fy_b = foot_radius_eff * sa + dy - oy;
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

// Ease into stance, then start the live gait at vx mm/s forward (negative
// walks backward) using gait `g` (0 tripod, 1 ripple, 2 wave).  Speed is
// clamped to a sane bench range.
void startWalking(float vx, int g) {
  g_auto = false;
  setGait(g);
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
  say(String("OK walking ") + String(g_vx, 0) + " mm/s, " + gaitName(g_gait)
      + " gait -- any key or C to stop");
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

// --- Dance moves -----------------------------------------------------------
// Eight whole-body party tricks.  Each runs as a live 50 Hz mode exactly like
// the walk (any key stops it).  Every move starts by easing into the planted
// stance at the remembered standing height, then ramps its amplitude in over
// ~1.5 s so nothing snaps.  Trigger by letter (V/O/B/T) or `M <n>` (1..8):
//
//   V/M1 stadium wave -- a crest of raised femur+tibia travels around the
//                        body azimuth, like a sports-crowd wave
//   O/M2 say hi       -- weight shifts back onto the rear four legs, both
//                        FRONT legs lift up and wave side-to-side
//   B/M3 hula         -- feet stay planted, the body sways in a 22 mm circle
//   T/M4 twist & dip  -- the body twists side-to-side on its yaws while
//                        bouncing up and down
//   M5   tripod march -- lift one tripod (CAD legs 0/2/4) high while the other
//                        (1/3/5) stays planted, then swap; always 3 feet down
//   M6   boogie       -- the alternating tripod stomp plus a body weight-shift
//                        sway so it grooves between steps
//   M7   pinwheel     -- a narrow raised-leg crest circles the body while every
//                        yaw twists in unison, so it looks like it spins
//   M8   freakout     -- fast alternating tripod hops with a body bounce and
//                        yaw flailing on the airborne legs
//   M9   pogo         -- feet planted, body bounces straight up/down + yaw wag
//   M10  cancan       -- a crest of STRAIGHT-leg kicks travels the body
//   M11  corkscrew    -- body grinds in a tilted circle (sway + vertical bob)
//   M12  shimmy       -- fast little side-to-side shake, yaws counter-rotating
//   M13  twist-stomp  -- alternating tripod stomp while the body twists L/R
//   M14  tippy-taps   -- a very narrow crest skitters round as rapid toe taps
//   M15  disco point  -- weight back, one front leg points up and sweeps an arc
//   M16  rave         -- grand finale: hops + bounce + twist + flailing (wildest)
//   -- gentle/slow set (small + slow, safe to leave running) --
//   M17  breathe      -- very slow, small body rise/fall
//   M18  sway         -- slow gentle side-to-side body lean
//   M19  nod          -- slow forward/back body lean (a gentle bow)
//   M20  slow wave    -- a gentle low crest drifts slowly around the body

// Stance hip/knee the dances modulate around (filled by startDance()).
float g_base_hip = STANCE_HIP_DEG, g_base_knee = STANCE_KNEE_DEG;

// Joint angles for wired leg i that keep its foot PLANTED at the stance
// footprint while the body is displaced by (ox, oy) mm in chassis X/Y AND the
// body sits at height `z` (foot stays on the ground, so a different z just
// raises/lowers the body) -- the same world->yaw-frame math the gait uses.
// False = out of IK reach.
bool plantedLegAnglesZ(int i, float ox, float oy, float z,
                       float& yaw_deg, float& hip_deg, float& knee_deg) {
  float ca = leg_cos[i], sa = leg_sin[i];
  float R  = LEG_RADIAL + stanceFootX(g_stand_z);   // foot stays where stance put it
  float rx = (R * ca - ox) - LEG_RADIAL * ca;
  float ry = (R * sa - oy) - LEG_RADIAL * sa;
  float x_yaw =  ca * rx + sa * ry;
  float y_yaw = -sa * rx + ca * ry;
  float p, k;
  if (!legIK(sqrtf(x_yaw * x_yaw + y_yaw * y_yaw), z, p, k))
    return false;
  yaw_deg  = degrees(atan2f(y_yaw, x_yaw));
  hip_deg  = degrees(p);
  knee_deg = degrees(k);
  return true;
}

// Planted-foot angles at the remembered standing height (the common case).
bool plantedLegAngles(int i, float ox, float oy,
                      float& yaw_deg, float& hip_deg, float& knee_deg) {
  return plantedLegAnglesZ(i, ox, oy, g_stand_z, yaw_deg, hip_deg, knee_deg);
}

void startDance(int n) {
  g_auto = false; g_walk = false; g_dance = 0;
  // Capture the stance angles this dance modulates around / returns to.
  float dest[18];
  if (stanceDestAtZ(g_stand_z, dest)) { g_base_hip = dest[1]; g_base_knee = dest[2]; }
  else { g_base_hip = STANCE_HIP_DEG; g_base_knee = STANCE_KNEE_DEG; }
  poseStance();                 // plant the feet first (eased)
  g_dance_phase   = 0.0f;
  g_dance_elapsed = 0.0f;
  last_step_us = micros();
  g_dance = n;
  static const char* names[] =
      {"", "V stadium wave", "O hi-wave (front legs)", "B hula sway", "T twist & dip",
       "M5 tripod march", "M6 tripod boogie", "M7 pinwheel", "M8 freakout",
       "M9 pogo", "M10 cancan", "M11 corkscrew", "M12 shimmy",
       "M13 twist-stomp", "M14 tippy-taps", "M15 disco point", "M16 rave",
       "M17 breathe", "M18 sway", "M19 nod", "M20 slow wave"};
  const char* nm = (n >= 1 && n <= 20) ? names[n] : "?";
  say(String("OK dancing: ") + nm + " -- any key stops");
}

// Raised-cosine lift pulse: 1.0 when the cycle phase is at `center`, easing to
// 0 by `hw` radians either side (and 0 beyond).  Two tripods half a cycle apart
// with hw < PI/2 therefore never lift at the same time -- there is always a
// double-support window, so an alternating-tripod move stays statically stable.
static inline float legPulse(float phase, float center, float hw) {
  float d = wrap2pi(phase - center);     // 0..2pi
  if (d > (float)PI) d = 2.0f * (float)PI - d;   // fold to 0..pi
  if (d >= hw) return 0.0f;
  return 0.5f * (1.0f + cosf(d / hw * (float)PI));
}

// One 50 Hz dance update (mirrors stepGait's timing contract).
void stepDance(float dt) {
  g_dance_elapsed += dt;
  float ramp = fminf(g_dance_elapsed / 1.5f, 1.0f);   // global amplitude ease-in

  switch (g_dance) {

  case 1: {   // V -- stadium wave: a raised-leg crest circles the body
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 2.4f);
    for (int i = 0; i < 6; ++i) {
      // Crest position is in CAD azimuth space so it travels smoothly
      // AROUND the body even though the wiring order is side-by-side.
      float az = (WIRED_TO_CAD_LEG[i] + 0.5f) * (float)PI / 3.0f;
      float c  = 0.5f * (1.0f + cosf(g_dance_phase - az));
      float bump = c * c * c * ramp;     // cube narrows the crest to ~1-2 legs
      writeJoint(i * 3 + 0, 0.0f);
      writeJoint(i * 3 + 1, g_base_hip  + bump * (-70.0f - g_base_hip));
      writeJoint(i * 3 + 2, g_base_knee + bump * (  5.0f - g_base_knee));
    }
  } break;

  case 2: {   // O -- weight back, lift the two FRONT legs and wave them
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 1.2f);
    float shift = fminf(g_dance_elapsed / 0.8f, 1.0f);              // weight back first
    float arms  = clampf((g_dance_elapsed - 0.7f) / 1.0f, 0.0f, 1.0f); // then arms up
    for (int i = 0; i < 6; ++i) {
      // Everyone tracks the 25 mm body shift while planted...
      float yd = 0.0f, hd = g_base_hip, kd = g_base_knee;
      plantedLegAngles(i, -25.0f * shift, 0.0f, yd, hd, kd);
      if (i == 0 || i == 3) {
        // ...but the front pair (wired 0 = right-front, 3 = left-front)
        // blends from planted into a raised wave, half a cycle apart.
        float ph = g_dance_phase + ((i == 3) ? (float)PI : 0.0f);
        float wy = 18.0f * sinf(ph);                  // side-to-side wave
        float wh = -55.0f;                            // femur raised high
        float wk = 15.0f + 25.0f * sinf(ph);          // knee flaps along
        yd += arms * (wy - yd);
        hd += arms * (wh - hd);
        kd += arms * (wk - kd);
      }
      writeJoint(i * 3 + 0, yd);
      writeJoint(i * 3 + 1, hd);
      writeJoint(i * 3 + 2, kd);
    }
  } break;

  case 3: {   // B -- hula: feet planted, body centre sways in a circle
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 2.0f);
    float ox = 22.0f * ramp * cosf(g_dance_phase);
    float oy = 22.0f * ramp * sinf(g_dance_phase);
    for (int i = 0; i < 6; ++i) {
      float yd, hd, kd;
      if (plantedLegAngles(i, ox, oy, yd, hd, kd)) {
        writeJoint(i * 3 + 0, yd);
        writeJoint(i * 3 + 1, hd);
        writeJoint(i * 3 + 2, kd);
      }
    }
  } break;

  case 4: {   // T -- twist & dip: yaws twist the body while it bounces
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 1.6f);
    float twist = 20.0f * ramp * sinf(g_dance_phase);
    float dip   = 14.0f * ramp * 0.5f * (1.0f - cosf(2.0f * g_dance_phase));
    float z = g_stand_z + dip;     // less negative = body dips DOWN
    float p, k;
    if (legIK(stanceFootX(z), z, p, k)) {
      for (int i = 0; i < 6; ++i) {
        writeJoint(i * 3 + 0, twist);
        writeJoint(i * 3 + 1, degrees(p));
        writeJoint(i * 3 + 2, degrees(k));
      }
    }
  } break;

  case 5: {   // M5 -- tripod march: lift one tripod (CAD legs 0/2/4) high while
              // the other (1/3/5) stays planted, then swap, and so on.  hw<PI/2
              // keeps a double-support gap so it never lifts all six at once.
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 1.5f);
    const float hw = 0.40f * (float)PI;
    for (int i = 0; i < 6; ++i) {
      int   grp    = WIRED_TO_CAD_LEG[i] & 1;           // two interleaved tripods
      float center = grp ? (float)PI : 0.0f;
      float lift   = legPulse(g_dance_phase, center, hw) * ramp;
      writeJoint(i * 3 + 0, 10.0f * lift * sinf(2.0f * g_dance_phase));   // little kick
      writeJoint(i * 3 + 1, g_base_hip  + lift * (-70.0f - g_base_hip));
      writeJoint(i * 3 + 2, g_base_knee + lift * (  8.0f - g_base_knee));
    }
  } break;

  case 6: {   // M6 -- tripod boogie: same alternating stomp, but the body also
              // sways side-to-side (planted feet track it) so it grooves.
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 1.15f);
    const float hw = 0.45f * (float)PI;
    float oy = 20.0f * ramp * cosf(g_dance_phase);     // lean toward the loaded tripod
    for (int i = 0; i < 6; ++i) {
      int   grp    = WIRED_TO_CAD_LEG[i] & 1;
      float center = grp ? (float)PI : 0.0f;
      float lift   = legPulse(g_dance_phase, center, hw) * ramp;
      float yd, hd, kd;
      if (!plantedLegAngles(i, 0.0f, oy, yd, hd, kd)) { yd = 0.0f; hd = g_base_hip; kd = g_base_knee; }
      if (lift > 0.001f) {                              // blend the lifted tripod up
        float wy = 16.0f * sinf(2.0f * g_dance_phase);
        float wh = -72.0f, wk = 25.0f * lift;
        yd += lift * (wy - yd);
        hd += lift * (wh - hd);
        kd += lift * (wk - kd);
      }
      writeJoint(i * 3 + 0, yd);
      writeJoint(i * 3 + 1, hd);
      writeJoint(i * 3 + 2, kd);
    }
  } break;

  case 7: {   // M7 -- pinwheel: a narrow raised-leg crest circles the body while
              // every leg's yaw twists in unison, so it looks like it spins.
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 1.6f);
    float twist = 28.0f * ramp * sinf(2.0f * (float)PI * g_dance_elapsed / 1.1f);
    for (int i = 0; i < 6; ++i) {
      float az   = (WIRED_TO_CAD_LEG[i] + 0.5f) * (float)PI / 3.0f;
      float c    = 0.5f * (1.0f + cosf(g_dance_phase - az));
      float bump = c * c * ramp;
      writeJoint(i * 3 + 0, twist);
      writeJoint(i * 3 + 1, g_base_hip  + bump * (-72.0f - g_base_hip));
      writeJoint(i * 3 + 2, g_base_knee + bump * (  0.0f - g_base_knee));
    }
  } break;

  case 8: {   // M8 -- freakout: fast alternating tripod hops + body bounce + yaw
              // flailing on the airborne legs.  The wildest one (max amplitude).
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 0.7f);
    const float hw = 0.42f * (float)PI;
    float dip = 16.0f * ramp * 0.5f * (1.0f - cosf(2.0f * g_dance_phase));   // 2 bounces/cycle
    float z   = g_stand_z + dip;
    float pz, kz;  bool ok = legIK(stanceFootX(z), z, pz, kz);
    for (int i = 0; i < 6; ++i) {
      int   grp    = WIRED_TO_CAD_LEG[i] & 1;
      float center = grp ? (float)PI : 0.0f;
      float lift   = legPulse(g_dance_phase, center, hw) * ramp;
      float hd = ok ? degrees(pz) : g_base_hip;
      float kd = ok ? degrees(kz) : g_base_knee;
      float yd;
      if (lift > 0.001f) {                              // airborne: flail
        float fl = sinf(6.0f * g_dance_phase);
        yd = 30.0f * lift * fl;
        hd += lift * (-78.0f - hd);
        kd += lift * (45.0f + 30.0f * fl - kd);
      } else {
        yd = 8.0f * ramp * sinf(3.0f * g_dance_phase); // planted: jitter the yaw
      }
      writeJoint(i * 3 + 0, yd);
      writeJoint(i * 3 + 1, hd);
      writeJoint(i * 3 + 2, kd);
    }
  } break;

  case 9: {   // M9 -- pogo: feet planted, body bounces straight up/down in
              // unison with a little yaw wag.
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 0.6f);
    float z   = g_stand_z + 16.0f * ramp * sinf(g_dance_phase);
    float wag = 6.0f * ramp * sinf(2.0f * g_dance_phase);
    for (int i = 0; i < 6; ++i) {
      float yd, hd, kd;
      if (plantedLegAnglesZ(i, 0.0f, 0.0f, z, yd, hd, kd)) {
        writeJoint(i * 3 + 0, wag);
        writeJoint(i * 3 + 1, hd);
        writeJoint(i * 3 + 2, kd);
      }
    }
  } break;

  case 10: {  // M10 -- cancan: a crest of STRAIGHT-leg kicks travels the body
              // (knee extends as each leg kicks out, unlike the tucked wave).
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 2.2f);
    for (int i = 0; i < 6; ++i) {
      float az   = (WIRED_TO_CAD_LEG[i] + 0.5f) * (float)PI / 3.0f;
      float c    = 0.5f * (1.0f + cosf(g_dance_phase - az));
      float bump = c * c * c * ramp;
      writeJoint(i * 3 + 0, bump * 25.0f * sinf(2.0f * g_dance_phase));   // kick sideways
      writeJoint(i * 3 + 1, g_base_hip  + bump * (-62.0f - g_base_hip));  // leg up
      writeJoint(i * 3 + 2, g_base_knee + bump * (-15.0f - g_base_knee)); // knee STRAIGHT
    }
  } break;

  case 11: {  // M11 -- corkscrew: the body grinds in a tilted circle (hula
              // sway + vertical bob on the same phase).
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 1.8f);
    float ox = 18.0f * ramp * cosf(g_dance_phase);
    float oy = 18.0f * ramp * sinf(g_dance_phase);
    float z  = g_stand_z + 10.0f * ramp * cosf(g_dance_phase);
    for (int i = 0; i < 6; ++i) {
      float yd, hd, kd;
      if (plantedLegAnglesZ(i, ox, oy, z, yd, hd, kd)) {
        writeJoint(i * 3 + 0, yd);
        writeJoint(i * 3 + 1, hd);
        writeJoint(i * 3 + 2, kd);
      }
    }
  } break;

  case 12: {  // M12 -- shimmy: fast little side-to-side body shake with the
              // yaws counter-rotating so it looks like it's vibrating.
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 0.35f);
    float oy    = 12.0f * ramp * sinf(g_dance_phase);
    float twist = 10.0f * ramp * sinf(g_dance_phase);
    for (int i = 0; i < 6; ++i) {
      float yd, hd, kd;
      if (plantedLegAngles(i, 0.0f, oy, yd, hd, kd)) {
        writeJoint(i * 3 + 0, yd + twist);
        writeJoint(i * 3 + 1, hd);
        writeJoint(i * 3 + 2, kd);
      }
    }
  } break;

  case 13: {  // M13 -- twist-stomp: alternating tripod stomp while the WHOLE
              // body twists left/right on the planted feet.
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 1.2f);
    const float hw = 0.42f * (float)PI;
    float twist = 22.0f * ramp * sinf(g_dance_phase);
    for (int i = 0; i < 6; ++i) {
      int   grp    = WIRED_TO_CAD_LEG[i] & 1;
      float center = grp ? (float)PI : 0.0f;
      float lift   = legPulse(g_dance_phase, center, hw) * ramp;
      writeJoint(i * 3 + 0, twist);
      writeJoint(i * 3 + 1, g_base_hip  + lift * (-70.0f - g_base_hip));
      writeJoint(i * 3 + 2, g_base_knee + lift * (  5.0f - g_base_knee));
    }
  } break;

  case 14: {  // M14 -- tippy-taps: a single very narrow crest skitters quickly
              // around the body so it looks like rapid little toe taps.
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 1.0f);
    for (int i = 0; i < 6; ++i) {
      float az   = (WIRED_TO_CAD_LEG[i] + 0.5f) * (float)PI / 3.0f;
      float c    = 0.5f * (1.0f + cosf(g_dance_phase - az));
      float c2   = c * c;  float bump = c2 * c2 * c2 * ramp;   // c^6: very narrow
      writeJoint(i * 3 + 0, 0.0f);
      writeJoint(i * 3 + 1, g_base_hip  + bump * (-48.0f - g_base_hip));
      writeJoint(i * 3 + 2, g_base_knee + bump * ( 20.0f - g_base_knee));
    }
  } break;

  case 15: {  // M15 -- disco point: weight shifts back, ONE front leg points
              // up straight and sweeps a big arc (Saturday-night-fever).
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 2.4f);
    float shift = fminf(g_dance_elapsed / 0.8f, 1.0f);
    float armup = clampf((g_dance_elapsed - 0.7f) / 1.0f, 0.0f, 1.0f);
    for (int i = 0; i < 6; ++i) {
      float yd = 0.0f, hd = g_base_hip, kd = g_base_knee;
      plantedLegAngles(i, -22.0f * shift, 0.0f, yd, hd, kd);
      if (i == 0) {                          // wired 0 = right-front: the pointer
        float py = 30.0f * sinf(g_dance_phase);     // big slow arc sweep
        float ph = -72.0f;                          // raised high
        float pk = -15.0f;                          // knee straight (pointing)
        yd += armup * (py - yd);
        hd += armup * (ph - hd);
        kd += armup * (pk - kd);
      }
      writeJoint(i * 3 + 0, yd);
      writeJoint(i * 3 + 1, hd);
      writeJoint(i * 3 + 2, kd);
    }
  } break;

  case 16: {  // M16 -- rave: the grand finale.  Fast alternating tripod hops +
              // body bounce + full-body twist + airborne flailing.  Everything.
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 0.55f);
    const float hw = 0.45f * (float)PI;
    float dip = 18.0f * ramp * 0.5f * (1.0f - cosf(2.0f * g_dance_phase));
    float z   = g_stand_z + dip;
    float pz, kz;  bool ok = legIK(stanceFootX(z), z, pz, kz);
    float twist = 30.0f * ramp * sinf(3.0f * g_dance_phase);
    for (int i = 0; i < 6; ++i) {
      int   grp    = WIRED_TO_CAD_LEG[i] & 1;
      float center = grp ? (float)PI : 0.0f;
      float lift   = legPulse(g_dance_phase, center, hw) * ramp;
      float hd = ok ? degrees(pz) : g_base_hip;
      float kd = ok ? degrees(kz) : g_base_knee;
      float yd = twist;
      if (lift > 0.001f) {
        float fl = sinf(8.0f * g_dance_phase);
        yd  = twist + 30.0f * lift * fl;
        hd += lift * (-80.0f - hd);
        kd += lift * (50.0f + 30.0f * fl - kd);
      }
      writeJoint(i * 3 + 0, yd);
      writeJoint(i * 3 + 1, hd);
      writeJoint(i * 3 + 2, kd);
    }
  } break;

  // ---- gentle / slow set (small amplitude, low speed -- safe to leave running)
  case 17: {  // M17 -- breathe: very slow, small body rise/fall (calm).
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 3.0f);
    float z = g_stand_z + 8.0f * ramp * sinf(g_dance_phase);
    for (int i = 0; i < 6; ++i) {
      float yd, hd, kd;
      if (plantedLegAnglesZ(i, 0.0f, 0.0f, z, yd, hd, kd)) {
        writeJoint(i * 3 + 0, 0.0f);
        writeJoint(i * 3 + 1, hd);
        writeJoint(i * 3 + 2, kd);
      }
    }
  } break;

  case 18: {  // M18 -- sway: slow gentle side-to-side body lean.
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 2.5f);
    float oy = 12.0f * ramp * sinf(g_dance_phase);
    for (int i = 0; i < 6; ++i) {
      float yd, hd, kd;
      if (plantedLegAngles(i, 0.0f, oy, yd, hd, kd)) {
        writeJoint(i * 3 + 0, yd);
        writeJoint(i * 3 + 1, hd);
        writeJoint(i * 3 + 2, kd);
      }
    }
  } break;

  case 19: {  // M19 -- nod: slow forward/back body lean, like a gentle bow.
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 2.5f);
    float ox = 12.0f * ramp * sinf(g_dance_phase);
    for (int i = 0; i < 6; ++i) {
      float yd, hd, kd;
      if (plantedLegAngles(i, ox, 0.0f, yd, hd, kd)) {
        writeJoint(i * 3 + 0, yd);
        writeJoint(i * 3 + 1, hd);
        writeJoint(i * 3 + 2, kd);
      }
    }
  } break;

  case 20: {  // M20 -- slow wave: a gentle low crest drifts slowly around.
    g_dance_phase = wrap2pi(g_dance_phase + 2.0f * (float)PI * dt / 3.0f);
    for (int i = 0; i < 6; ++i) {
      float az   = (WIRED_TO_CAD_LEG[i] + 0.5f) * (float)PI / 3.0f;
      float c    = 0.5f * (1.0f + cosf(g_dance_phase - az));
      float bump = c * c * c * ramp;                 // narrow, gentle crest
      writeJoint(i * 3 + 0, 0.0f);
      writeJoint(i * 3 + 1, g_base_hip  + bump * (-40.0f - g_base_hip));  // modest lift
      writeJoint(i * 3 + 2, g_base_knee + bump * ( 30.0f - g_base_knee));
    }
  } break;
  }
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
  s += " ch ";     s += jointChannel(j);
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
  // initBoard() above wakes a freshly-seen board; if we are DISARMED, force
  // its channels straight back to full-off so a rescan never energizes a
  // servo while the robot is meant to be limp.
  if (!g_armed) disarmOutputs();
}

void printHelp() {
  String s = "OK prototype_servo_test\n";
  s += "  (boots DISARMED -- all servos LIMP/no PWM until you send ARM)\n";
  s += "  ARM       enable servo outputs (nothing moves on arm; then send P)\n";
  s += "  X/DISARM  DISARM: cut all PWM, servos limp -- EMERGENCY STOP\n";
  s += "  G         GO: start/resume the auto sweep (all joints)\n";
  s += "  <ENTER>   test NEXT joint\n";
  s += "  N <j>     test joint j (0..17), full sweep\n";
  s += "  Q <j> [d] quick wiggle joint j +/-d deg (default 8) -- is it listening?\n";
  s += "  # <j> <deg> set ONE joint j (0..17) to an absolute angle and HOLD it\n";
  s += "  L <leg>   test leg (0..5), all 3 joints\n";
  s += "  A         test ALL 18 in sequence\n";
  s += "  R         repeat last joint\n";
  s += "  C         centre all + stop\n";
  s += "  X         DISARM: turn OFF all motors (relax/limp, e-stop)\n";
  s += "  F <frac>  sweep amplitude 0.1..1.0\n";
  s += "  S         slow/fast sweep toggle\n";
  s += "  I         scan I2C bus (are the 0x40/0x41 drivers seen?)\n";
  s += "  -- whole-body --\n";
  s += "  U         legs UP over head\n";
  s += "  P         STAND UP: staged low-current rise (fold->tripod lift->spread)\n";
  s += "  CROUCH    pre-lift TUCK: ease into the low feet-tucked crouch (no lift;\n";
  s += "            then send P to stand). gradual, tripod-staggered, ARM-gated\n";
  s += "  SIT       graceful sit-down: slew-limited lower stand->low rest (stays\n";
  s += "            held/powered so it settles, doesn't smash down). then DISARM\n";
  s += "  SETTLE    SIT then DISARM: lower gently, THEN cut power (web relax/off)\n";
  s += "  D [mm]    descend feet gradually + REMEMBER ground height (any key stops;\n";
  s += "            feet tuck inward as they drop so the legs can lift the body)\n";
  s += "  Z [mm]    show (or set) the remembered standing foot Z\n";
  s += "  $ [mm]    show (or set) stand-up tuck-in radius (feet pull-in; 90..200)\n";
  s += "  W [mm/s] [t|r|w|q]  walk: t=tripod r=ripple w=wave q=tetrapod\n";
  s += "            e.g. W 20, W r, W 25 w. tripod=fast, wave=most stable.\n";
  s += "            any key/C stops\n";
  s += "  J vx vy w [g]  live analog drive (gamepad bridge): mm/s, mm/s,\n";
  s += "            rad/s, optional gait 0/1/2. steers without restarting\n";
  s += "  K [mm]    swing lift height (lower=steadier). live while walking\n";
  s += "  E [x y]   COM lean trim mm (+x fwd,+y left) to cancel imbalance\n";
  s += "  -- dances (any key stops; M <n> picks any by number 1..20) --\n";
  s += "  V (M1) wave  O (M2) say hi  B (M3) hula  T (M4) twist & dip\n";
  s += "  M5 march  M6 boogie  M7 pinwheel  M8 freakout  M9 pogo\n";
  s += "  M10 cancan  M11 corkscrew  M12 shimmy  M13 twist-stomp\n";
  s += "  M14 tippy-taps  M15 disco point  M16 rave (finale)\n";
  s += "  gentle/slow: M17 breathe  M18 sway  M19 nod  M20 slow wave\n";
  s += "  ?         help";
  say(s);
}

// Case-insensitive full-line token match, ignoring trailing spaces.  Used for
// the multi-char ARM / DISARM safety tokens (kept dependency-free -- no
// strcasecmp, which isn't reliably available on this Zephyr/newlib-nano core).
static bool tokenIs(const char* s, const char* tok) {
  while (*tok) {
    char a = *s++, b = *tok++;
    if (a >= 'a' && a <= 'z') a -= 32;
    if (b >= 'a' && b <= 'z') b -= 32;
    if (a != b) return false;
  }
  while (*s == ' ') ++s;
  return *s == '\0';
}

// Does this command letter drive/energize a servo?  Used by the DISARMED
// backstop to refuse motion until the robot is ARMed.  '\0' (bare ENTER) is
// the "test next joint" command, so it counts as motion too.
static bool isMotionCmd(char c) {
  if (c == '\0') return true;
  for (const char* q = "GUPDWJVOBTMCRANQL#"; *q; ++q)
    if (*q == c) return true;
  return false;
}

void handleLine(char* line) {
  g_seen_input = true;   // silence the idle heartbeat once we hear from the user
  g_dance = 0;           // ANY command stops a running dance (V/O/B/T/M restart one)
  while (*line == ' ') ++line;

  // Safety tokens (multi-char), checked before the single-letter dispatch.
  if (tokenIs(line, "ARM"))    { arm();     return; }
  if (tokenIs(line, "DISARM")) { relaxAll(); return; }

  // Graceful settle tokens (multi-char, all MOTION -> gated behind ARM).  These
  // are the safe "come down gently" moves; DISARM above stays the INSTANT e-stop
  // (cut PWM now).  Every letter is already taken, hence word tokens.
  //   CROUCH  pre-lift tuck: ease into the low, feet-tucked crouch (no lift);
  //           then send P (Stand) to lift from there.
  //   SIT     graceful sit-down: slew-limited lower from standing to a low rest
  //           pose so it settles instead of smashing down (stays powered/held).
  //   SETTLE  SIT, THEN DISARM -- the web "relax/disarm/off" while standing:
  //           lower gently first, only THEN cut power (go limp).
  if (tokenIs(line, "CROUCH") || tokenIs(line, "SIT") || tokenIs(line, "SETTLE")) {
    if (!g_armed) { say("DISARMED -- servos off. Send ARM to enable, then retry."); return; }
    if      (tokenIs(line, "CROUCH")) crouchDown();
    else if (tokenIs(line, "SIT"))    sitDown();
    else                              { sitDown(); relaxAll(); }
    return;
  }

  char cmd = line[0];
  char* p = line + 1;

  // DISARMED backstop: until the human ARMs, refuse anything that could drive
  // a servo.  Arming is the single gate.  Diagnostics + parameter/config
  // commands (?, H, I, Y, S, F, Z, $, E, K) and X/DISARM still work while limp.
  // The motion word tokens (CROUCH, SIT, SETTLE) are ARM-gated above.
  if (!g_armed && isMotionCmd(cmd)) {
    say("DISARMED -- servos off. Send ARM to enable, then retry.");
    return;
  }

  // `G` (go) RESUMES the hands-free auto sweep; `W` starts walking; every
  // other command takes manual control by leaving both modes.  Stopping
  // the gait/sweep is just sending any other key (e.g. C to centre).
  if (cmd == 'G') { g_auto = true; g_walk = false; say("OK auto"); return; }
  if (cmd == 'U') { poseLegsUp();      return; }
  if (cmd == 'P') { standUp();         return; }   // staged low-current stand-up
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
  if (cmd == '$') {
    // `$` prints the stand-up rise tuck-in radius; `$ <mm>` sets + saves it.
    // Smaller = feet tucked tighter under the body during the rise (shorter
    // arm = more lift leverage); clamped to the reachable band up to the
    // sprawled stance radius.  Only affects the RISE; the final stance radius
    // stays governed by stanceFootX().  Mirrors the `Z` calibration command.
    while (*p == ' ') ++p;
    if (*p == '\0') {
      say(String("OK stand tuck-in radius = ") + String(g_stand_tuck_r, 1) + " mm");
    } else {
      g_stand_tuck_r = clampf(atof(p), 90.0f, 200.0f);
      saveStandTuck();
      say(String("OK stand tuck-in radius set to ") + String(g_stand_tuck_r, 1)
          + " mm (smaller = tucked tighter; send P to apply)");
    }
    return;
  }
  if (cmd == 'W') {
    // `W [mm/s] [t|r|w|q]` -- speed then optional gait letter (tripod/ripple/
    // wave/tetrapod).  `W` alone = tripod @ 35; `W r` = ripple; `W 20 w` =
    // wave @ 20 mm/s; `W q` = tetrapod; negative speed walks backward.
    float vx = atof(p);
    int gait = 0;
    for (char* q = p; *q; ++q) {
      if (*q == 'r' || *q == 'R') { gait = 1; break; }
      if (*q == 'w' || *q == 'W') { gait = 2; break; }
      if (*q == 'q' || *q == 'Q') { gait = 3; break; }
      if (*q == 't' || *q == 'T') { gait = 0; break; }
    }
    if (vx == 0.0f) vx = (gait == 2) ? 18.0f : (gait == 1) ? 28.0f
                       : (gait == 3) ? 30.0f : 35.0f;
    startWalking(vx, gait);
    return;
  }
  if (cmd == 'J') {
    // Live analog drive (used by the Linux gamepad bridge):
    //   J <vx> <vy> <omega> [gait]
    // vx/vy mm/s (chassis +X fwd, +Y left), omega rad/s (+ = turn left),
    // optional gait 0/1/2/3.  Updates the live gait velocities WITHOUT
    // restarting it (smooth steering) and starts walking if not already.
    // Nothing is printed back -- it's streamed at ~20 Hz.
    char* e = p;
    float vx = strtod(p, &e);
    float vy = strtod(e, &e);
    float w  = strtod(e, &e);
    while (*e == ' ') ++e;
    if (*e >= '0' && *e <= '3') { int gg = *e - '0'; if (gg != g_gait) setGait(gg); }
    g_vx    = clampf(vx, -120.0f, 120.0f);
    g_vy    = clampf(vy, -120.0f, 120.0f);
    g_omega = clampf(w,  -1.5f,   1.5f);
    if (!g_walk) {                       // first packet: ease into the gait
      g_auto = false;
      foot_radius_eff = LEG_RADIAL + stanceFootX(g_stand_z);
      poseStance();
      g_phase = 0.0f; g_elapsed = 0.0f;
      last_step_us = micros();
      g_walk = true;
    }
    return;
  }
  if (cmd == 'E') {
    // `E` prints the COM trim; `E <x> <y>` sets it (mm, chassis +X fwd /
    // +Y left).  Lean the body to cancel a known load imbalance -- e.g. the
    // wire bundle pulling it to one side.  Applies on the next/again walk.
    while (*p == ' ') ++p;
    if (*p == '\0') {
      say(String("OK COM trim x=") + String(g_com_x, 1) + " y=" + String(g_com_y, 1)
          + " mm (gain " + String(g_com_gain, 2) + ")");
    } else {
      char* endp = p;
      g_com_x = strtod(p, &endp);
      g_com_y = strtod(endp, NULL);
      say(String("OK COM trim set x=") + String(g_com_x, 1) + " y=" + String(g_com_y, 1)
          + " mm (takes effect while walking)");
    }
    return;
  }
  if (cmd == 'K') {
    // `K` prints the swing lift; `K <mm>` sets it.  Lower = steadier (less
    // body rock per step); higher = clears taller obstacles.
    while (*p == ' ') ++p;
    if (*p == '\0') {
      say(String("OK swing lift = ") + String(g_lift, 1) + " mm");
    } else {
      g_lift = clampf(atof(p), 4.0f, 45.0f);
      say(String("OK swing lift set to ") + String(g_lift, 1) + " mm");
    }
    return;
  }
  if (cmd == 'V') { startDance(1); return; }   // stadium wave around the body
  if (cmd == 'O') { startDance(2); return; }   // say hi with the front legs
  if (cmd == 'B') { startDance(3); return; }   // hula body sway
  if (cmd == 'T') { startDance(4); return; }   // twist & dip
  if (cmd == 'M') {                            // M <n> -- dance by number (1..20)
    int n = atoi(p);
    if (n < 1 || n > 20) { say("ERR dance 1..20"); return; }
    startDance(n);
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
  if (cmd == '#') {
    // `# <joint> <deg>` -- set ONE joint to an absolute angle and HOLD it,
    // clamped to that axis's safe range.  Per-servo debug positioning used
    // by the web panel's Debug page.  This mirrors the servo BRIDGE's
    // `J <joint> <deg>`; here `J` is the live analog-drive command, so a
    // distinct `#` prefix is used to avoid the collision.  Modes are already
    // cleared above, so nothing re-drives the joint after we set it.
    char* e = p;
    long  j   = strtol(p, &e, 10);
    float deg = strtod(e, NULL);
    if (j < 0 || j >= 18) { say("ERR joint"); return; }
    writeJoint((int)j, deg);
    float lo, hi; axisLimits((int)j, lo, hi);
    say(String("OK joint ") + (int)j + " -> " + String(clampf(deg, lo, hi), 1)
        + " deg (" + axisName((int)j) + " leg " + ((int)j / 3) + ")");
    return;
  }
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
  loadStandTuck();   // override the rise tuck-in radius if one was saved (AVR EEPROM only)
  // i2cScan() probes the bus AND initialises every board it finds (wake +
  // 50 Hz).  Re-send `I` any time you connect a board after boot and it
  // will be configured -- no reflash, no power-cycle needed.  Because we are
  // DISARMED (g_armed == false), i2cScan() also forces every channel to
  // full-off at the end, so waking the boards does NOT energize any servo.
  i2cScan();

  // SAFE BOOT: start DISARMED -- do NOT auto-centre (the old boot-time
  // centreAll() slammed all 18 joints at once, the mechanical-stress +
  // stall-current spike that melted power wiring).  Instead force every
  // channel to no-PWM so all servos sit LIMP until a human sends `ARM`.
  relaxAll();     // sets g_armed=false + all channels full-off (limp)
  printHelp();

  // Nothing is driven at boot.  Put the feet where you want them, then from
  // the web page (or a monitor) send ARM to enable outputs, then P to stand.
  say("READY -- DISARMED & limp at boot. Send ARM to enable outputs, then P to stand.");
}

void loop() {
  pumpSerial();

  if (g_dance) {
    digitalWrite(LED_BUILTIN, (millis() / 150) & 1);   // disco blink
    unsigned long now = micros();
    unsigned long elapsed = now - last_step_us;
    if (elapsed >= STEP_PERIOD_US) {
      last_step_us = now;
      stepDance((float)elapsed * 1e-6f);
    }
    return;
  }

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
