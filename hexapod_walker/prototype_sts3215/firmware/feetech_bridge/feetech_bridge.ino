/*
  Feetech MCU bridge — Uno Q

  Hardware
    Serial  (USART1, D0/D1) @ 1 Mbps  → FE-URT UART header (TX–TX, RX–RX, GND)
    Serial1 (LPUART1)       @ 921600  → Linux /dev/ttyHS1 (stop arduino-router)
    Wire    (I2C2)          SDA/SCL   → GY-521 MPU-6050 (header D20/D21, 3V3)
    ST7789 TFT (bitbang SPI): SCL D13, SDA D11, RST D8, DC D7, CS D10
      (TFT "SDA" is MOSI bitbang — not the I²C SDA pin)

  Autonomy (no host required):
    - Boot: splash + "linux Ns" ticker + dim "Zz" hub badge until the
      host first speaks.
    - Host watchdog: >12 s of silence after first contact → red "web
      lost Ns" + red-X hub badge; >30 s → AUTO-LIMP (torque off, IDs
      2..19) so a dead brain can't leave motors fighting. Cleared as
      soon as the host talks again.

  Line protocol (ASCII, \\n-terminated). Host owns the conversation.

    HELLO                         → HELLO feetech_bridge
    PING <id>                     → OK <id> | ERR
    SCAN                          → OK <id>,<id>,... | OK
    T <id> <0|1>                  → OK | ERR
    TA <0|1>                      → OK          (torque IDs 2..19)
    RP <id>                       → OK <pos> | ERR
    R1 <id> <addr>                → OK <byte> | ERR
    R2 <id> <addr>                → OK <word> | ERR
    W1 <id> <addr> <val>          → OK | ERR
    W2 <id> <addr> <val>          → OK | ERR
    WP <id> <pos> <spd> <acc>     → OK | ERR   (WritePosEx)
    UL <id> / LK <id>             → OK | ERR   (EEPROM unlock / lock)
    SW <n> <id> <pos> <spd> <acc> ... → OK | ERR
    PWR                           → OK <n> <I_mA> <V10> <load10>
    DI                            → OK          (hard-reinit TFT)
    DT                            → OK          (TFT self-test: W/R/G/B)
    DX line|line|...              → OK <n> <I_mA> <V10> <load10>
                                    (status text + motor-current logo lights)
    DJ <pct>|title|l1|l2|l3|foot  → OK          (full-screen job panel:
                                    26-char rows + progress bar; pct −1 =
                                    no bar; next DX restores the schematic)
    I2CSCAN                       → OK <hex>,... | OK   (Wire bus scan)
    IMU                           → OK 0x68 | ERR ...   (wake + WHO_AM_I)
    IMUR                          → OK ax ay az gx gy gz temp
                                    (raw int16; temp = raw TEMP_OUT)

  Binary fast path (same UART):
    A5 5A 'W' n  {id u8, pos i16le, spd u16le, acc u8}×n  xor
    → OK\\n | ERR\\n
    A5 5A 'F' n  {id u8}×n  xor     (n=0 → IDs 2..19)
    → A5 5A 'f' n {id,ok,pos_i16,spd_i16,load_u16,volt_u8,temp_u8,mov_u8,cur_i16}×n xor
      (Feetech syncRead of PRESENT_POSITION..CURRENT; FeedBack fallback)
    A5 5A 'P' n  {id u8}×n  xor     (n=0 → IDs 2..19)
    → A5 5A 'p' n {id,ok,pos_i16}×n xor
      (Feetech syncRead of PRESENT_POSITION only)
*/

#include <Wire.h>
#include <SCServo.h>
#include "st7789_tft.h"

// MPU-6050 / GY-521 on header Wire (SDA/SCL = D20/D21). Not Linux I²C.
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t MPU_REG_SMPLRT_DIV = 0x19;
static const uint8_t MPU_REG_CONFIG = 0x1A;
static const uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
static const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU_REG_WHO_AM_I = 0x75;
static bool mpuReady = false;

static const uint32_t BUS_BAUD = 1000000UL;
static const uint32_t HOST_BAUD = 921600UL;  // Linux /dev/ttyHS1 must match
static const uint8_t ID_LO = 2;
static const uint8_t ID_HI = 19;
static const uint8_t MAX_N = 18;
// syncRead: PRESENT_POSITION_L (56) .. PRESENT_CURRENT_H (70) = 15 bytes
static const uint8_t FB_MEM_LEN = 15;
static const uint8_t POS_MEM_LEN = 2;
static bool syncReadReady = false;
static uint8_t syncReadRxLen = 0;

SMS_STS sts;

// FTServo readSCS waits until the buffer is full or timeout — size must match
// the packet length we request, or short (pos-only) reads burn the whole timeout.
static void ensureSyncRead(uint8_t rxLen) {
  if (syncReadReady && syncReadRxLen == rxLen) return;
  if (syncReadReady) sts.syncReadEnd();
  sts.syncReadBegin(MAX_N, rxLen, 20);
  syncReadReady = true;
  syncReadRxLen = rxLen;
}

static char lineBuf[640];
static uint16_t lineLen = 0;

static void hostPrint(const __FlashStringHelper *s) { Serial1.print(s); }
static void hostPrint(const char *s) { Serial1.print(s); }
static void hostPrint(int v) { Serial1.print(v); }
static void hostPrintln() { Serial1.println(); }

static void replyOk() { Serial1.println(F("OK")); }
static void replyErr() { Serial1.println(F("ERR")); }

static bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuReadRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom(MPU_ADDR, n);
  if (got != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static int16_t be16(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static bool mpuWake() {
  // Clear sleep bit; clock = internal 8 MHz.
  if (!mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x00)) return false;
  delay(50);
  // Light defaults: 1 kHz sample / DLPF ~44 Hz / ±250 dps / ±2 g.
  mpuWriteReg(MPU_REG_SMPLRT_DIV, 0x07);
  mpuWriteReg(MPU_REG_CONFIG, 0x03);
  mpuWriteReg(MPU_REG_GYRO_CONFIG, 0x00);
  mpuWriteReg(MPU_REG_ACCEL_CONFIG, 0x00);
  return true;
}

static void cmdI2cScan() {
  Serial1.print(F("OK"));
  bool any = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial1.print(any ? ',' : ' ');
      Serial1.print(F("0x"));
      if (addr < 16) Serial1.print('0');
      Serial1.print(addr, HEX);
      any = true;
    }
  }
  Serial1.println();
}

// Returns WHO_AM_I on success, 0 on failure. Sets mpuReady.
static uint8_t mpuEnsureReady() {
  uint8_t who = 0;
  if (!mpuReadRegs(MPU_REG_WHO_AM_I, &who, 1)) {
    mpuReady = false;
    return 0;
  }
  if (who != 0x68 && who != 0x70) {
    // 0x68 = MPU-6050; some clones report 0x70.
    mpuReady = false;
    return 0;
  }
  if (!mpuWake()) {
    mpuReady = false;
    return 0;
  }
  mpuReady = true;
  return who;
}

static void cmdImu() {
  uint8_t who = 0;
  if (!mpuReadRegs(MPU_REG_WHO_AM_I, &who, 1)) {
    mpuReady = false;
    Serial1.println(F("ERR no_ack"));
    return;
  }
  if (who != 0x68 && who != 0x70) {
    mpuReady = false;
    Serial1.print(F("ERR whoami 0x"));
    if (who < 16) Serial1.print('0');
    Serial1.println(who, HEX);
    return;
  }
  if (!mpuWake()) {
    mpuReady = false;
    Serial1.println(F("ERR wake"));
    return;
  }
  mpuReady = true;
  Serial1.print(F("OK 0x"));
  if (who < 16) Serial1.print('0');
  Serial1.println(who, HEX);
}

static void cmdImuRead() {
  if (!mpuReady && mpuEnsureReady() == 0) {
    Serial1.println(F("ERR no_ack"));
    return;
  }
  uint8_t raw[14];
  if (!mpuReadRegs(MPU_REG_ACCEL_XOUT_H, raw, 14)) {
    mpuReady = false;
    Serial1.println(F("ERR read"));
    return;
  }
  Serial1.print(F("OK "));
  Serial1.print(be16(raw + 0));
  Serial1.print(' ');
  Serial1.print(be16(raw + 2));
  Serial1.print(' ');
  Serial1.print(be16(raw + 4));
  Serial1.print(' ');
  Serial1.print(be16(raw + 8));
  Serial1.print(' ');
  Serial1.print(be16(raw + 10));
  Serial1.print(' ');
  Serial1.print(be16(raw + 12));
  Serial1.print(' ');
  Serial1.println(be16(raw + 6));  // TEMP_OUT
}

static int skipSpaces(const char *s, int i) {
  while (s[i] == ' ' || s[i] == '\t') i++;
  return i;
}

static bool parseInt(const char *s, int &i, long &out) {
  // Manual parse — Zephyr Arduino libc may not link strtol.
  i = skipSpaces(s, i);
  if (s[i] == '\0') return false;
  bool neg = false;
  if (s[i] == '-') {
    neg = true;
    i++;
  } else if (s[i] == '+') {
    i++;
  }
  if (s[i] < '0' || s[i] > '9') return false;
  long v = 0;
  while (s[i] >= '0' && s[i] <= '9') {
    v = v * 10 + (s[i] - '0');
    i++;
  }
  out = neg ? -v : v;
  return true;
}

static void cmdHello() {
  Serial1.println(F("HELLO feetech_bridge"));
}

static void cmdPing(long id) {
  if (id < 1 || id > 253) {
    replyErr();
    return;
  }
  int got = sts.Ping((int)id);
  if (!sts.getLastError() && got == (int)id) {
    Serial1.print(F("OK "));
    Serial1.println((int)id);
  } else {
    replyErr();
  }
}

static void cmdScan() {
  Serial1.print(F("OK"));
  bool any = false;
  for (int id = ID_LO; id <= ID_HI; id++) {
    int got = sts.Ping(id);
    if (!sts.getLastError() && got == id) {
      Serial1.print(any ? ',' : ' ');
      Serial1.print(id);
      any = true;
    }
  }
  // Also report factory ID 1 if present (setup aid).
  {
    int got = sts.Ping(1);
    if (!sts.getLastError() && got == 1) {
      Serial1.print(any ? ',' : ' ');
      Serial1.print(1);
      any = true;
    }
  }
  Serial1.println();
}

// Bus power snapshot for the TFT / web. Current LSB ≈ 6.5 mA.
static void cmdPwr() {
  long sumRaw = 0;
  int n = 0;
  int vSum = 0;
  int maxLoad = 0;
  for (int id = ID_LO; id <= ID_HI; id++) {
    int cur = sts.ReadCurrent(id);
    if (sts.getLastError()) continue;
    // STS current is signed in some firmwares; use magnitude.
    if (cur < 0) cur = -cur;
    sumRaw += cur;
    n++;
    int v = sts.ReadVoltage(id);
    if (!sts.getLastError() && v > 0) vSum += v;
    int ld = sts.ReadLoad(id);
    if (!sts.getLastError()) {
      int mag = ld & 0x3FF;
      if (mag > maxLoad) maxLoad = mag;
    }
  }
  long iMa = (sumRaw * 65L + 5) / 10;  // raw * 6.5 mA
  int v10 = (n > 0) ? (vSum / n) : 0;  // deci-volts average
  Serial1.print(F("OK "));
  Serial1.print(n);
  Serial1.print(' ');
  Serial1.print(iMa);
  Serial1.print(' ');
  Serial1.print(v10);
  Serial1.print(' ');
  Serial1.println(maxLoad);  // load tenths of %
}

static void cmdTorque(long id, long on) {
  if (id < 1 || id > 253) {
    replyErr();
    return;
  }
  sts.EnableTorque((u8)id, on ? 1 : 0);
  replyOk();
}

static void cmdTorqueAll(long on) {
  u8 en = on ? 1 : 0;
  for (int id = ID_LO; id <= ID_HI; id++) {
    sts.EnableTorque((u8)id, en);
  }
  replyOk();
}

static void cmdReadPos(long id) {
  if (id < 1 || id > 253) {
    replyErr();
    return;
  }
  int pos = sts.ReadPos((int)id);
  if (sts.getLastError()) {
    replyErr();
    return;
  }
  Serial1.print(F("OK "));
  Serial1.println(pos);
}

static void doSyncWrite(uint8_t n, uint8_t *ids, s16 *pos, u16 *spd, u8 *acc) {
  if (n == 0 || n > MAX_N) {
    replyErr();
    return;
  }
  sts.SyncWritePosEx(ids, n, pos, spd, acc);
  replyOk();
}

static void cmdSyncWriteAscii(const char *s, int i) {
  long nLong = 0;
  if (!parseInt(s, i, nLong) || nLong < 1 || nLong > MAX_N) {
    replyErr();
    return;
  }
  uint8_t n = (uint8_t)nLong;
  uint8_t ids[MAX_N];
  s16 pos[MAX_N];
  u16 spd[MAX_N];
  u8 acc[MAX_N];
  for (uint8_t k = 0; k < n; k++) {
    long id = 0, p = 0, speed = 0, a = 0;
    if (!parseInt(s, i, id) || !parseInt(s, i, p) ||
        !parseInt(s, i, speed) || !parseInt(s, i, a)) {
      replyErr();
      return;
    }
    ids[k] = (uint8_t)id;
    pos[k] = (s16)p;
    spd[k] = (u16)speed;
    acc[k] = (u8)a;
  }
  doSyncWrite(n, ids, pos, spd, acc);
}

static void handleLine(char *line) {
  // Trim CR
  for (char *p = line; *p; p++) {
    if (*p == '\r') *p = '\0';
  }
  if (line[0] == '\0') return;

  if (strcmp(line, "HELLO") == 0) {
    cmdHello();
    return;
  }
  if (strcmp(line, "SCAN") == 0) {
    cmdScan();
    return;
  }
  if (strcmp(line, "I2CSCAN") == 0) {
    cmdI2cScan();
    return;
  }
  if (strcmp(line, "IMU") == 0) {
    cmdImu();
    return;
  }
  if (strcmp(line, "IMUR") == 0) {
    cmdImuRead();
    return;
  }

  int i = 0;
  // Commands with args
  if (strncmp(line, "PING", 4) == 0) {
    i = 4;
    long id = 0;
    if (!parseInt(line, i, id)) {
      replyErr();
      return;
    }
    cmdPing(id);
    return;
  }
  if (line[0] == 'T' && line[1] == 'A') {
    i = 2;
    long on = 0;
    if (!parseInt(line, i, on)) {
      replyErr();
      return;
    }
    cmdTorqueAll(on);
    return;
  }
  if (line[0] == 'T' && (line[1] == ' ' || line[1] == '\t')) {
    i = 1;
    long id = 0, on = 0;
    if (!parseInt(line, i, id) || !parseInt(line, i, on)) {
      replyErr();
      return;
    }
    cmdTorque(id, on);
    return;
  }
  if (strncmp(line, "RP", 2) == 0) {
    i = 2;
    long id = 0;
    if (!parseInt(line, i, id)) {
      replyErr();
      return;
    }
    cmdReadPos(id);
    return;
  }
  if (strncmp(line, "R1", 2) == 0) {
    i = 2;
    long id = 0, addr = 0;
    if (!parseInt(line, i, id) || !parseInt(line, i, addr)) {
      replyErr();
      return;
    }
    int v = sts.readByte((u8)id, (u8)addr);
    if (sts.getLastError()) {
      replyErr();
      return;
    }
    Serial1.print(F("OK "));
    Serial1.println(v);
    return;
  }
  if (strncmp(line, "R2", 2) == 0) {
    i = 2;
    long id = 0, addr = 0;
    if (!parseInt(line, i, id) || !parseInt(line, i, addr)) {
      replyErr();
      return;
    }
    int v = sts.readWord((u8)id, (u8)addr);
    if (sts.getLastError()) {
      replyErr();
      return;
    }
    Serial1.print(F("OK "));
    Serial1.println(v);
    return;
  }
  if (strncmp(line, "W1", 2) == 0) {
    i = 2;
    long id = 0, addr = 0, val = 0;
    if (!parseInt(line, i, id) || !parseInt(line, i, addr) ||
        !parseInt(line, i, val)) {
      replyErr();
      return;
    }
    sts.writeByte((u8)id, (u8)addr, (u8)val);
    if (sts.getLastError()) {
      replyErr();
      return;
    }
    replyOk();
    return;
  }
  if (strncmp(line, "W2", 2) == 0) {
    i = 2;
    long id = 0, addr = 0, val = 0;
    if (!parseInt(line, i, id) || !parseInt(line, i, addr) ||
        !parseInt(line, i, val)) {
      replyErr();
      return;
    }
    sts.writeWord((u8)id, (u8)addr, (u16)val);
    if (sts.getLastError()) {
      replyErr();
      return;
    }
    replyOk();
    return;
  }
  if (strncmp(line, "WP", 2) == 0) {
    i = 2;
    long id = 0, pos = 0, spd = 0, acc = 0;
    if (!parseInt(line, i, id) || !parseInt(line, i, pos) ||
        !parseInt(line, i, spd) || !parseInt(line, i, acc)) {
      replyErr();
      return;
    }
    sts.WritePosEx((u8)id, (s16)pos, (u16)spd, (u8)acc);
    if (sts.getLastError()) {
      replyErr();
      return;
    }
    replyOk();
    return;
  }
  if (strncmp(line, "UL", 2) == 0) {
    i = 2;
    long id = 0;
    if (!parseInt(line, i, id)) {
      replyErr();
      return;
    }
    sts.unLockEprom((u8)id);
    replyOk();
    return;
  }
  if (strncmp(line, "LK", 2) == 0) {
    i = 2;
    long id = 0;
    if (!parseInt(line, i, id)) {
      replyErr();
      return;
    }
    sts.LockEprom((u8)id);
    replyOk();
    return;
  }
  if (strncmp(line, "SW", 2) == 0) {
    cmdSyncWriteAscii(line, 2);
    return;
  }
  if (strncmp(line, "PWR", 3) == 0) {
    cmdPwr();
    return;
  }
  if (strncmp(line, "DI", 2) == 0 &&
      (line[2] == '\0' || line[2] == ' ')) {
    // Always hard-reset the panel — reseating the ribbon leaves ``ready``
    // true in RAM while the ST7789 itself is blank again.
    tft::reinit();
    replyOk();
    return;
  }
  if (strncmp(line, "DT", 2) == 0 &&
      (line[2] == '\0' || line[2] == ' ')) {
    tft::selfTest();
    replyOk();
    return;
  }
  if (strncmp(line, "DX", 2) == 0) {
    // Read per-joint current (mA) for logo glow, then paint status text.
    int ma[18];
    long sumRaw = 0;
    int n = 0;
    int vSum = 0;
    int maxLoad = 0;
    for (int k = 0; k < 18; k++) ma[k] = 0;
    for (int id = ID_LO; id <= ID_HI; id++) {
      int idx = id - (int)ID_LO;
      int cur = sts.ReadCurrent(id);
      if (sts.getLastError()) continue;
      if (cur < 0) cur = -cur;
      sumRaw += cur;
      ma[idx] = (int)((cur * 65L + 5) / 10);  // raw × 6.5 mA
      n++;
      int v = sts.ReadVoltage(id);
      if (!sts.getLastError() && v > 0) vSum += v;
      int ld = sts.ReadLoad(id);
      if (!sts.getLastError()) {
        int mag = ld & 0x3FF;
        if (mag > maxLoad) maxLoad = mag;
      }
    }
    long iMa = (sumRaw * 65L + 5) / 10;
    int v10 = (n > 0) ? (vSum / n) : 0;
    int j = skipSpaces(line, 2);
    tft::pushPanel(line + j, ma, n, iMa, v10, maxLoad);
    Serial1.print(F("OK "));
    Serial1.print(n);
    Serial1.print(' ');
    Serial1.print(iMa);
    Serial1.print(' ');
    Serial1.print(v10);
    Serial1.print(' ');
    Serial1.println(maxLoad);
    return;
  }
  if (strncmp(line, "DJ", 2) == 0) {
    // Job panel: full-screen text + progress bar during calibration/demo
    // jobs. Pure display — no servo reads, so it stays cheap while the
    // Linux side is hammering the bus. Next DX returns to the schematic.
    int j = skipSpaces(line, 2);
    tft::pushJob(line + j);
    replyOk();
    return;
  }

  replyErr();
}

// Binary frame reader state
static uint8_t binState = 0;  // 0 idle, 1 got A5, 2 got 5A, 3 got cmd, reading
static uint8_t binCmd = 0;
static uint8_t binN = 0;
static uint8_t binNeed = 0;
static uint8_t binGot = 0;
static uint8_t binPayload[MAX_N * 6];
static uint8_t binXor = 0;

static void binPutU16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

static void binPutI16(uint8_t *p, int16_t v) {
  binPutU16(p, (uint16_t)v);
}

static void sendBinHeader(uint8_t cmd, uint8_t n, uint8_t &x) {
  Serial1.write((uint8_t)0xA5);
  Serial1.write((uint8_t)0x5A);
  Serial1.write(cmd);
  Serial1.write(n);
  x = (uint8_t)(cmd ^ n);
}

static void sendBinByte(uint8_t b, uint8_t &x) {
  Serial1.write(b);
  x ^= b;
}

static void fillDefaultIds(uint8_t *ids, uint8_t &n) {
  n = 0;
  for (uint8_t id = ID_LO; id <= ID_HI && n < MAX_N; id++) {
    ids[n++] = id;
  }
}

static bool decodeFbFromMem(int16_t &pos, int16_t &spd, uint16_t &load,
                            uint8_t &volt, uint8_t &temp, uint8_t &mov,
                            int16_t &cur) {
  // Mem[] already filled by FeedBack / mirrored sync packet via Read*(-1).
  pos = (int16_t)sts.ReadPos(-1);
  spd = (int16_t)sts.ReadSpeed(-1);
  int ld = sts.ReadLoad(-1);
  load = (uint16_t)(ld < 0 ? -ld : ld);
  volt = (uint8_t)sts.ReadVoltage(-1);
  temp = (uint8_t)sts.ReadTemper(-1);
  mov = (uint8_t)sts.ReadMove(-1);
  cur = (int16_t)sts.ReadCurrent(-1);
  return true;
}

// Decode 15-byte syncRead payload (addr 56..70). End=0 → lo,hi in Mem.
static void decodeFbPacket(const uint8_t *rx, int16_t &pos, int16_t &spd,
                           uint16_t &load, uint8_t &volt, uint8_t &temp,
                           uint8_t &mov, int16_t &cur) {
  auto le16 = [](uint8_t lo, uint8_t hi) -> int {
    return (int)((uint16_t)lo | ((uint16_t)hi << 8));
  };
  auto signed15 = [](int w) -> int16_t {
    if (w & (1 << 15)) w = -(w & ~(1 << 15));
    return (int16_t)w;
  };
  auto signed10 = [](int w) -> int {
    if (w & (1 << 10)) w = -(w & ~(1 << 10));
    return w;
  };
  pos = signed15(le16(rx[0], rx[1]));
  spd = signed15(le16(rx[2], rx[3]));
  int ld = signed10(le16(rx[4], rx[5]));
  load = (uint16_t)(ld < 0 ? -ld : ld);
  volt = rx[6];
  temp = rx[7];
  mov = rx[10];  // reg 66; rx[8..9] and rx[11..12] unused
  cur = signed15(le16(rx[13], rx[14]));
}

// Full state: one syncRead TX for all IDs (15-byte block), FeedBack fallback.
static void cmdBulkFeedback(uint8_t n, const uint8_t *ids) {
  uint8_t useN = n;
  uint8_t useIds[MAX_N];
  if (useN == 0) {
    fillDefaultIds(useIds, useN);
  } else {
    for (uint8_t i = 0; i < useN; i++) useIds[i] = ids[i];
  }

  bool usedSync = false;
  if (useN > 0) {
    ensureSyncRead(FB_MEM_LEN);
    sts.syncReadPacketTx(useIds, useN, SMS_STS_PRESENT_POSITION_L, FB_MEM_LEN);
    usedSync = true;
  }

  uint8_t x = 0;
  sendBinHeader('f', useN, x);
  for (uint8_t k = 0; k < useN; k++) {
    uint8_t id = useIds[k];
    uint8_t ok = 0;
    int16_t pos = 0, spd = 0, cur = 0;
    uint16_t load = 0;
    uint8_t volt = 0, temp = 0, mov = 0;
    uint8_t rx[FB_MEM_LEN];

    if (usedSync && sts.syncReadPacketRx(id, rx)) {
      ok = 1;
      decodeFbPacket(rx, pos, spd, load, volt, temp, mov, cur);
    } else {
      int nLen = sts.FeedBack((int)id);
      if (nLen <= 0) nLen = sts.FeedBack((int)id);
      if (nLen > 0) {
        ok = 1;
        decodeFbFromMem(pos, spd, load, volt, temp, mov, cur);
      }
    }

    uint8_t rec[13];
    rec[0] = id;
    rec[1] = ok;
    binPutI16(rec + 2, pos);
    binPutI16(rec + 4, spd);
    binPutU16(rec + 6, load);
    rec[8] = volt;
    rec[9] = temp;
    rec[10] = mov;
    binPutI16(rec + 11, cur);
    for (uint8_t i = 0; i < 13; i++) sendBinByte(rec[i], x);
  }
  Serial1.write(x);
}

// Position-only via syncRead (2 bytes); ReadPos fallback.
static void cmdBulkPositions(uint8_t n, const uint8_t *ids) {
  uint8_t useN = n;
  uint8_t useIds[MAX_N];
  if (useN == 0) {
    fillDefaultIds(useIds, useN);
  } else {
    for (uint8_t i = 0; i < useN; i++) useIds[i] = ids[i];
  }

  bool usedSync = false;
  if (useN > 0) {
    ensureSyncRead(POS_MEM_LEN);
    sts.syncReadPacketTx(useIds, useN, SMS_STS_PRESENT_POSITION_L, POS_MEM_LEN);
    usedSync = true;
  }

  uint8_t x = 0;
  sendBinHeader('p', useN, x);
  for (uint8_t k = 0; k < useN; k++) {
    uint8_t id = useIds[k];
    int16_t pos = 0;
    uint8_t ok = 0;
    uint8_t rx[POS_MEM_LEN];
    if (usedSync && sts.syncReadPacketRx(id, rx)) {
      int w = (int)((uint16_t)rx[0] | ((uint16_t)rx[1] << 8));
      if (w & (1 << 15)) w = -(w & ~(1 << 15));
      pos = (int16_t)w;
      ok = 1;
    } else {
      int p = sts.ReadPos((int)id);
      if (!sts.getLastError()) {
        ok = 1;
        pos = (int16_t)p;
      }
    }
    sendBinByte(id, x);
    sendBinByte(ok, x);
    sendBinByte((uint8_t)((uint16_t)pos & 0xFF), x);
    sendBinByte((uint8_t)((uint16_t)pos >> 8), x);
  }
  Serial1.write(x);
}

static void handleBinaryFrame() {
  if (binCmd == 'W') {
    if (binN == 0 || binN > MAX_N || binGot != binN * 6) {
      replyErr();
      return;
    }
    uint8_t ids[MAX_N];
    s16 pos[MAX_N];
    u16 spd[MAX_N];
    u8 acc[MAX_N];
    for (uint8_t k = 0; k < binN; k++) {
      const uint8_t *p = binPayload + k * 6;
      ids[k] = p[0];
      pos[k] = (s16)((uint16_t)p[1] | ((uint16_t)p[2] << 8));
      spd[k] = (u16)((uint16_t)p[3] | ((uint16_t)p[4] << 8));
      acc[k] = p[5];
    }
    doSyncWrite(binN, ids, pos, spd, acc);
    return;
  }
  if (binCmd == 'F') {
    if (binN > MAX_N || binGot != binN) {
      replyErr();
      return;
    }
    cmdBulkFeedback(binN, binPayload);
    return;
  }
  if (binCmd == 'P') {
    if (binN > MAX_N || binGot != binN) {
      replyErr();
      return;
    }
    cmdBulkPositions(binN, binPayload);
    return;
  }
  replyErr();
}

static void feedHostByte(uint8_t b) {
  // Binary frames A5 5A — otherwise ASCII lines.
  if (binState == 0) {
    if (b == 0xA5) {
      binState = 1;
      return;
    }
    if (b == '\n') {
      lineBuf[lineLen] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
      return;
    }
    if (b == '\r') return;
    if (lineLen + 1 < sizeof(lineBuf)) {
      lineBuf[lineLen++] = (char)b;
    } else {
      lineLen = 0;  // overflow — drop
    }
    return;
  }

  if (binState == 1) {
    binState = (b == 0x5A) ? 2 : 0;
    if (binState == 0 && b == 0xA5) binState = 1;
    return;
  }
  if (binState == 2) {
    binCmd = b;
    binState = 3;
    return;
  }
  if (binState == 3) {
    binN = b;
    binGot = 0;
    binXor = (uint8_t)(binCmd ^ binN);
    if (binCmd == 'W') {
      binNeed = (uint8_t)(binN * 6);
      if (binN == 0 || binN > MAX_N) binNeed = 0;
    } else if (binCmd == 'F' || binCmd == 'P') {
      // n==0 means "default IDs 2..19" — no id payload.
      binNeed = (binN > MAX_N) ? 0 : binN;
    } else {
      binNeed = 0;
    }
    binState = 4;
    return;
  }
  if (binState == 4) {
    if (binGot < binNeed) {
      binPayload[binGot++] = b;
      binXor ^= b;
      return;
    }
    // checksum
    bool okN = (binCmd == 'F' || binCmd == 'P')
                   ? (binN <= MAX_N && binGot == binNeed)
                   : (binN > 0 && binN <= MAX_N && binGot == binNeed);
    if (b == binXor && okN) {
      handleBinaryFrame();
    } else {
      replyErr();
    }
    binState = 0;
    return;
  }
}

void setup() {
  // 1) Speak to Linux first (short settle only — do not wait on the TFT).
  Serial1.begin(HOST_BAUD);
  delay(30);
  Serial1.println(F("HELLO feetech_bridge"));

  // 2) Hexapod on screen ASAP — before Wire / servo UART / anything else.
  tft::bootSplash();

  // 3) Rest of the bridge.
  Serial.begin(BUS_BAUD);
  sts.pSerial = &Serial;
  ensureSyncRead(FB_MEM_LEN);
  Wire.begin();  // header SDA/SCL (D20/D21) → MPU-6050
}

static bool hostSeen = false;
static unsigned long lastHostMs = 0;
// Linux refreshes the panel every ~2 s; this much silence after first
// contact means the web service (or the SoC) died.
static const unsigned long HOST_LOST_MS = 12000;
// After this much silence, cut all servo torque so a dead brain can't
// leave motors fighting/cooking (2026-08-06 incident). Well above the
// ~15-20 s a normal `systemctl restart hexapod-web` gap takes.
static const unsigned long HOST_LIMP_MS = 30000;
static bool autoLimped = false;

void loop() {
  if (Serial1.available() > 0) {
    hostSeen = true;
    lastHostMs = millis();
    autoLimped = false;
    do {
      feedHostByte((uint8_t)Serial1.read());
    } while (Serial1.available() > 0);
    return;
  }
  unsigned long now = millis();
  if (!hostSeen) {
    // Boot counter until Linux first speaks; afterwards the host owns
    // the panel (DX schematic / DJ job screens).
    tft::bootTick(now);
    return;
  }
  if (now - lastHostMs > HOST_LOST_MS) {
    static unsigned long lastWarnMs = 0;
    if (now - lastWarnMs >= 1000) {
      lastWarnMs = now;
      tft::hostLostTick((now - lastHostMs) / 1000UL);
    }
    if (!autoLimped && now - lastHostMs > HOST_LIMP_MS) {
      autoLimped = true;
      for (int id = ID_LO; id <= ID_HI; id++) {
        sts.EnableTorque((u8)id, 0);
      }
      tft::autoLimpPaint();
    }
  }
}
