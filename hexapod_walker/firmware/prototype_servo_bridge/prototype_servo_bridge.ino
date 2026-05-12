/*
  Prototype hexapod servo bridge
  --------------------------------

  Arduino Mega + 2x PCA9685 boards.  The Raspberry Pi (or laptop) sends
  newline-terminated serial commands:

      J <joint> <deg>      set one joint, joint=0..17, deg=-80..+80
      A <18 floats>        set all joints in degrees
      C                    centre all joints
      T <joint> <deg>      set trim for one joint
      P                    print current trim table
      ?                    help

  Joint order matches the simulation and docs:

      joint = leg * 3 + axis
      axis 0 = yaw, axis 1 = hip pitch, axis 2 = knee pitch

  PCA9685 routing:

      joints  0..15 -> board 0x40 channels 0..15
      joints 16..17 -> board 0x41 channels 0..1

  Power note:
      Servo V+ must come from the external 5-6 V BEC rails.  Do not power
      DS3225 servos from the Arduino 5 V pin.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1(0x40);
Adafruit_PWMServoDriver pwm2(0x41);

const float PWM_MIN_US = 500.0;    // DS3225 nominal -90 deg
const float PWM_MAX_US = 2500.0;   // DS3225 nominal +90 deg
const float ANGLE_LIMIT_DEG = 80.0;
const int SERVO_HZ = 50;

float trim_deg[18] = {
  0, 0, 0,  0, 0, 0,  0, 0, 0,
  0, 0, 0,  0, 0, 0,  0, 0, 0
};

float target_deg[18] = {
  0, 0, 0,  0, 0, 0,  0, 0, 0,
  0, 0, 0,  0, 0, 0,  0, 0, 0
};

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void writeJoint(int joint_idx, float angle_deg) {
  if (joint_idx < 0 || joint_idx >= 18) return;
  angle_deg = clampf(angle_deg, -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG);
  target_deg[joint_idx] = angle_deg;

  float corrected = clampf(angle_deg + trim_deg[joint_idx],
                           -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG);
  float us = PWM_MIN_US
           + (corrected + 90.0) / 180.0 * (PWM_MAX_US - PWM_MIN_US);

  Adafruit_PWMServoDriver& drv = (joint_idx < 16) ? pwm1 : pwm2;
  int chan = joint_idx % 16;
  drv.writeMicroseconds(chan, (int)(us + 0.5));
}

void centreAll() {
  for (int i = 0; i < 18; ++i) {
    writeJoint(i, 0.0);
  }
}

void printHelp() {
  Serial.println(F("OK prototype_servo_bridge"));
  Serial.println(F("Commands:"));
  Serial.println(F("  J <joint 0..17> <deg -80..80>"));
  Serial.println(F("  A <18 deg values>"));
  Serial.println(F("  C"));
  Serial.println(F("  T <joint 0..17> <trim_deg>"));
  Serial.println(F("  P"));
  Serial.println(F("  ?"));
}

void printTrims() {
  Serial.print(F("TRIM"));
  for (int i = 0; i < 18; ++i) {
    Serial.print(' ');
    Serial.print(trim_deg[i], 2);
  }
  Serial.println();
}

bool parseFloats(char* p, float* out, int count) {
  for (int i = 0; i < count; ++i) {
    while (*p == ' ') ++p;
    if (*p == '\0') return false;
    char* endp = nullptr;
    out[i] = strtod(p, &endp);
    if (endp == p) return false;
    p = endp;
  }
  return true;
}

void handleLine(char* line) {
  while (*line == ' ') ++line;
  char cmd = line[0];
  char* p = line + 1;

  if (cmd == '?' || cmd == 'H') {
    printHelp();
    return;
  }
  if (cmd == 'C') {
    centreAll();
    Serial.println(F("OK C"));
    return;
  }
  if (cmd == 'P') {
    printTrims();
    return;
  }
  if (cmd == 'J') {
    int joint = atoi(p);
    while (*p == ' ') ++p;
    while (*p && *p != ' ') ++p;
    float deg = atof(p);
    if (joint < 0 || joint >= 18) {
      Serial.println(F("ERR joint"));
      return;
    }
    writeJoint(joint, deg);
    Serial.print(F("OK J "));
    Serial.print(joint);
    Serial.print(' ');
    Serial.println(deg, 2);
    return;
  }
  if (cmd == 'T') {
    int joint = atoi(p);
    while (*p == ' ') ++p;
    while (*p && *p != ' ') ++p;
    float deg = atof(p);
    if (joint < 0 || joint >= 18) {
      Serial.println(F("ERR joint"));
      return;
    }
    trim_deg[joint] = clampf(deg, -30.0, 30.0);
    Serial.print(F("OK T "));
    Serial.print(joint);
    Serial.print(' ');
    Serial.println(trim_deg[joint], 2);
    writeJoint(joint, target_deg[joint]);
    return;
  }
  if (cmd == 'A') {
    float vals[18];
    if (!parseFloats(p, vals, 18)) {
      Serial.println(F("ERR A expects 18 floats"));
      return;
    }
    for (int i = 0; i < 18; ++i) {
      writeJoint(i, vals[i]);
    }
    Serial.println(F("OK A"));
    return;
  }

  Serial.println(F("ERR unknown"));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm1.begin();
  pwm2.begin();
  pwm1.setPWMFreq(SERVO_HZ);
  pwm2.setPWMFreq(SERVO_HZ);
  delay(50);
  centreAll();
  printHelp();
}

void loop() {
  static char buf[220];
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
