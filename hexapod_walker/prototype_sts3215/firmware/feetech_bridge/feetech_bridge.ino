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
    STREAM <0|1>                  → OK STREAM <0|1>  (free-run mode below)
    STREAM                        → OK STREAM <0|1>  (query)
    DBG                           → OK key=value ... (bridge counters)
    DBG RESET                     → OK key=value ... (reset then report)

  Binary fast path (same UART):
    A5 5A 'W' n  {id u8, pos i16le, spd u16le, acc u8}×n  xor
    → OK\\n | ERR\\n
    A5 5A 'F' n  {id u8}×n  xor     (n=0 → IDs 2..19)
    → A5 5A 'f' n {id,ok,pos_i16,spd_i16,load_u16,volt_u8,temp_u8,mov_u8,cur_i16}×n xor
      (Feetech syncRead of PRESENT_POSITION..CURRENT; FeedBack fallback)
    A5 5A 'P' n  {id u8}×n  xor     (n=0 → IDs 2..19)
    → A5 5A 'p' n {id,ok,pos_i16}×n xor
      (Feetech syncRead of PRESENT_POSITION only)
    A5 5A 'S' n  {id u8, pos i16le, spd u16le, acc u8}×n  xor
    → A5 5A 's' 18 {seq u16, pos_age_ms u16, imu_age_ms u16,
                    imu i16×7 (ax ay az gx gy gz temp),
                    18×{id u8, ok u8, pos i16le, spd i16le}} xor
      (SyncWrite the commands, then reply with the latest state
       snapshot — ONE host round-trip per control tick. n=0 = no
       write, snapshot only: positions + speed + IMU in one round
       trip for sense-compute-act loops.)

  STREAM mode (the 50–100 Hz feedback architecture, 2026-08-19):
    While STREAM 1 and the host line is idle, the MCU free-runs the
    servo bus itself: a pos+speed syncRead of IDs 2..19 plus an IMU
    read every pass (~150–250 Hz), and a full 15-byte state syncRead
    (load/volt/temp/moving/current) every FB_PERIOD_MS (~10 Hz).
    Results live in RAM caches. Host requests then return CACHED data
    with zero servo-bus wait: 'P', 'F', 'S', IMUR, PWR and DX all
    serve from the caches, so Linux never blocks on 18 servo replies
    inside its control loop. Host still owns the conversation — the
    MCU never pushes unsolicited bytes. Default OFF at boot; the
    Linux bus driver enables it right after HELLO.
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
// PRESENT_POSITION_L (56) .. PRESENT_SPEED_H (59) — the stream fast block.
static const uint8_t POS_SPD_MEM_LEN = 4;
static bool syncReadReady = false;
static uint8_t syncReadRxLen = 0;
static uint32_t syncReadTimeoutMs = 20;

SMS_STS sts;

// FTServo readSCS waits until the buffer is full or timeout — size must match
// the packet length we request, or short (pos-only) reads burn the whole
// timeout. Timeout is per missing servo: keep it short in stream mode so one
// dead ID cannot eat the control period (20 ms legacy, ~5 ms streaming).
static void ensureSyncRead(uint8_t rxLen, uint32_t timeoutMs) {
  if (syncReadReady && syncReadRxLen == rxLen
      && syncReadTimeoutMs == timeoutMs) return;
  if (syncReadReady) sts.syncReadEnd();
  sts.syncReadBegin(MAX_N, rxLen, timeoutMs);
  syncReadReady = true;
  syncReadRxLen = rxLen;
  syncReadTimeoutMs = timeoutMs;
}

// ---- STREAM mode state (free-running acquisition caches) ----
static bool streaming = false;
static int16_t posCache[MAX_N];
static int16_t spdCache[MAX_N];       // counts/s, sign-decoded
static uint8_t posOk[MAX_N];
static uint16_t posSeq = 0;
static unsigned long posStampMs = 0;
static int16_t imuCache[7];           // ax ay az gx gy gz temp (raw)
static bool imuCacheValid = false;
static unsigned long imuStampMs = 0;
static unsigned long imuRetryMs = 0;
static uint16_t fbLoad[MAX_N];        // magnitude, tenths of %
static uint8_t fbVolt[MAX_N];         // deci-volts
static uint8_t fbTemp[MAX_N];
static uint8_t fbMov[MAX_N];
static int16_t fbCur[MAX_N];          // raw, ×6.5 mA
static uint8_t fbOk[MAX_N];
static unsigned long fbStampMs = 0;
// Full-state pass cadence. Positions/speed/IMU refresh every pass
// (~150-250 Hz); current/load/volt/temp only need ~10 Hz.
static const unsigned long FB_PERIOD_MS = 100;
// During host-owned 100 Hz control, serve 'S' replies from the last fresh
// cache, then refresh pos/speed/IMU in the idle gap before the next tick.
// This keeps the read cadence near the control rate, with one-tick latency,
// while taking servo/I2C waits out of the host round-trip.
static const unsigned long HOST_S_REFRESH_DELAY_MS = 1;
static const unsigned long HOST_S_CONTROL_IDLE_MS = 30;
static const unsigned long HOST_S_MAX_CACHE_AGE_MS = 12;
static bool hostSRefreshPending = false;
static unsigned long hostSRefreshAtMs = 0;
static unsigned long hostSLastMs = 0;

static bool streaming_fb_fresh() {
  return streaming && fbStampMs != 0;
}

// Defined below (need sts / bin helpers first).
static void streamFastPass();
static void streamFullPass();
static void streamImuPass();
static void prepareSnapshotForHost();

// The host-UART RX ring is SMALLER than one 113-byte 'W'/'S' frame
// (measured 2026-08-19: a frame landing while the MCU is inside a 4-6 ms
// acquisition pass loses bytes). So passes must keep draining host bytes
// into the frame parser. Execution is DEFERRED: running a command
// mid-pass would collide on the half-duplex servo bus, so a completed
// frame is parked and run right after the pass.
static bool deferHostExec = false;
static uint8_t parkedKind = 0;  // 0 none, 1 binary frame, 2 ascii line
static void hostPump();         // drain host bytes into the parser
static void execParked();       // run a parked frame (servo bus free)

static char lineBuf[640];
static uint16_t lineLen = 0;

// ---- Bridge diagnostics: counters are read/reset by the DBG command. ----
static uint32_t dbgHostBytesSeen = 0;
static uint32_t dbgAsciiLinesCompleted = 0;
static uint32_t dbgAsciiLinesParked = 0;
static uint32_t dbgAsciiOkReplies = 0;
static uint32_t dbgAsciiErrReplies = 0;
static uint32_t dbgLineOverflow = 0;
static uint32_t dbgUnknownAscii = 0;
static uint32_t dbgBinFramesStarted = 0;
static uint32_t dbgBinFramesCompleted = 0;
static uint32_t dbgBinFramesParked = 0;
static uint32_t dbgBinFramesExecuted = 0;
static uint32_t dbgBinChecksumBad = 0;
static uint32_t dbgBinBadN = 0;
static uint32_t dbgBinBadCmd = 0;
static uint32_t dbgBinReplyHeaders = 0;
static uint32_t dbgDesyncResets = 0;
static uint32_t dbgSyncWriteCalls = 0;
static uint32_t dbgSyncWriteFailures = 0;
static uint32_t dbgStreamFastPasses = 0;
static uint32_t dbgStreamFullPasses = 0;
static uint32_t dbgStreamImuPasses = 0;
static uint32_t dbgStreamPosSlotFails = 0;
static uint32_t dbgStreamFbSlotFails = 0;
static uint32_t dbgHostSnapshotRequests = 0;
static uint32_t dbgHostSnapshotCacheHits = 0;
static uint32_t dbgHostSnapshotSyncRefreshes = 0;
static uint32_t dbgHostSnapshotAsyncRefreshes = 0;
static uint32_t dbgMaxFastPassUs = 0;
static uint32_t dbgMaxFullPassUs = 0;
static uint32_t dbgMaxImuPassUs = 0;
static uint32_t dbgMaxBinExecUs = 0;
static uint32_t dbgMaxParkedWaitUs = 0;
static uint32_t dbgLastBinFrameStartUs = 0;
static uint32_t dbgLastBinFrameCompleteUs = 0;
static uint32_t dbgBinParkedAtUs = 0;
static uint8_t dbgLastBinCmd = 0;
static uint8_t dbgLastBinN = 0;

static void dbgUpdateMax(uint32_t &dst, uint32_t val) {
  if (val > dst) dst = val;
}

static void dbgResetCounters() {
  dbgHostBytesSeen = 0;
  dbgAsciiLinesCompleted = 0;
  dbgAsciiLinesParked = 0;
  dbgAsciiOkReplies = 0;
  dbgAsciiErrReplies = 0;
  dbgLineOverflow = 0;
  dbgUnknownAscii = 0;
  dbgBinFramesStarted = 0;
  dbgBinFramesCompleted = 0;
  dbgBinFramesParked = 0;
  dbgBinFramesExecuted = 0;
  dbgBinChecksumBad = 0;
  dbgBinBadN = 0;
  dbgBinBadCmd = 0;
  dbgBinReplyHeaders = 0;
  dbgDesyncResets = 0;
  dbgSyncWriteCalls = 0;
  dbgSyncWriteFailures = 0;
  dbgStreamFastPasses = 0;
  dbgStreamFullPasses = 0;
  dbgStreamImuPasses = 0;
  dbgStreamPosSlotFails = 0;
  dbgStreamFbSlotFails = 0;
  dbgHostSnapshotRequests = 0;
  dbgHostSnapshotCacheHits = 0;
  dbgHostSnapshotSyncRefreshes = 0;
  dbgHostSnapshotAsyncRefreshes = 0;
  dbgMaxFastPassUs = 0;
  dbgMaxFullPassUs = 0;
  dbgMaxImuPassUs = 0;
  dbgMaxBinExecUs = 0;
  dbgMaxParkedWaitUs = 0;
  dbgLastBinFrameStartUs = 0;
  dbgLastBinFrameCompleteUs = 0;
  dbgBinParkedAtUs = 0;
  dbgLastBinCmd = 0;
  dbgLastBinN = 0;
}

static void dbgPrintKV(const __FlashStringHelper *key, uint32_t value) {
  Serial1.print(' ');
  Serial1.print(key);
  Serial1.print('=');
  Serial1.print(value);
}

static void dbgPrintKV8(const __FlashStringHelper *key, uint8_t value) {
  dbgPrintKV(key, (uint32_t)value);
}

static void cmdDbg(bool reset) {
  if (reset) dbgResetCounters();
  Serial1.print(F("OK"));
  dbgPrintKV(F("uptime_ms"), (uint32_t)millis());
  dbgPrintKV8(F("streaming"), streaming ? 1 : 0);
  dbgPrintKV(F("host_bytes_seen"), dbgHostBytesSeen);
  dbgPrintKV(F("ascii_lines_completed"), dbgAsciiLinesCompleted);
  dbgPrintKV(F("ascii_lines_parked"), dbgAsciiLinesParked);
  dbgPrintKV(F("ascii_ok_replies"), dbgAsciiOkReplies);
  dbgPrintKV(F("ascii_err_replies"), dbgAsciiErrReplies);
  dbgPrintKV(F("line_overflow"), dbgLineOverflow);
  dbgPrintKV(F("unknown_ascii"), dbgUnknownAscii);
  dbgPrintKV(F("bin_frames_started"), dbgBinFramesStarted);
  dbgPrintKV(F("bin_frames_completed"), dbgBinFramesCompleted);
  dbgPrintKV(F("bin_frames_parked"), dbgBinFramesParked);
  dbgPrintKV(F("bin_frames_executed"), dbgBinFramesExecuted);
  dbgPrintKV(F("bin_checksum_bad"), dbgBinChecksumBad);
  dbgPrintKV(F("bin_bad_n"), dbgBinBadN);
  dbgPrintKV(F("bin_bad_cmd"), dbgBinBadCmd);
  dbgPrintKV(F("bin_reply_headers"), dbgBinReplyHeaders);
  dbgPrintKV(F("desync_resets"), dbgDesyncResets);
  dbgPrintKV(F("syncwrite_calls"), dbgSyncWriteCalls);
  dbgPrintKV(F("syncwrite_failures"), dbgSyncWriteFailures);
  dbgPrintKV(F("stream_fast_passes"), dbgStreamFastPasses);
  dbgPrintKV(F("stream_full_passes"), dbgStreamFullPasses);
  dbgPrintKV(F("stream_imu_passes"), dbgStreamImuPasses);
  dbgPrintKV(F("stream_pos_slot_fails"), dbgStreamPosSlotFails);
  dbgPrintKV(F("stream_fb_slot_fails"), dbgStreamFbSlotFails);
  dbgPrintKV(F("host_snapshot_requests"), dbgHostSnapshotRequests);
  dbgPrintKV(F("host_snapshot_cache_hits"), dbgHostSnapshotCacheHits);
  dbgPrintKV(F("host_snapshot_sync_refreshes"), dbgHostSnapshotSyncRefreshes);
  dbgPrintKV(F("host_snapshot_async_refreshes"), dbgHostSnapshotAsyncRefreshes);
  dbgPrintKV(F("max_fast_pass_us"), dbgMaxFastPassUs);
  dbgPrintKV(F("max_full_pass_us"), dbgMaxFullPassUs);
  dbgPrintKV(F("max_imu_pass_us"), dbgMaxImuPassUs);
  dbgPrintKV(F("max_bin_exec_us"), dbgMaxBinExecUs);
  dbgPrintKV(F("max_parked_wait_us"), dbgMaxParkedWaitUs);
  dbgPrintKV8(F("last_bin_cmd"), dbgLastBinCmd);
  dbgPrintKV8(F("last_bin_n"), dbgLastBinN);
  Serial1.println();
}

static void hostPrint(const __FlashStringHelper *s) { Serial1.print(s); }
static void hostPrint(const char *s) { Serial1.print(s); }
static void hostPrint(int v) { Serial1.print(v); }
static void hostPrintln() { Serial1.println(); }

static void replyOk() {
  dbgAsciiOkReplies++;
  Serial1.println(F("OK"));
}

static void replyErr() {
  dbgAsciiErrReplies++;
  Serial1.println(F("ERR"));
}

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
  // STREAM mode: serve the background-refreshed cache (<= a few ms old)
  // instead of a fresh I2C transaction.
  if (streaming && imuCacheValid) {
    Serial1.print(F("OK "));
    for (uint8_t i = 0; i < 6; i++) {
      Serial1.print(imuCache[i]);
      Serial1.print(' ');
    }
    Serial1.println(imuCache[6]);
    return;
  }
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

// Per-servo current (mA, optional) + bus totals. In STREAM mode this is
// served from the ~10 Hz full-state cache — the legacy path is 54
// individual servo reads (~30-60 ms) that used to stall the control
// loop every TFT refresh.
static void gatherPowerStats(int *maPerJoint, int &n, long &iMa,
                             int &v10, int &maxLoad) {
  long sumRaw = 0;
  int vSum = 0;
  n = 0;
  maxLoad = 0;
  if (maPerJoint) {
    for (int k = 0; k < (int)MAX_N; k++) maPerJoint[k] = 0;
  }
  if (streaming_fb_fresh()) {
    for (int k = 0; k < (int)MAX_N; k++) {
      if (!fbOk[k]) continue;
      int cur = fbCur[k];
      if (cur < 0) cur = -cur;
      sumRaw += cur;
      if (maPerJoint) maPerJoint[k] = (int)((cur * 65L + 5) / 10);
      n++;
      if (fbVolt[k] > 0) vSum += fbVolt[k];
      int mag = fbLoad[k] & 0x3FF;
      if (mag > maxLoad) maxLoad = mag;
    }
  } else {
    for (int id = ID_LO; id <= ID_HI; id++) {
      int idx = id - (int)ID_LO;
      int cur = sts.ReadCurrent(id);
      if (sts.getLastError()) continue;
      // STS current is signed in some firmwares; use magnitude.
      if (cur < 0) cur = -cur;
      sumRaw += cur;
      if (maPerJoint) maPerJoint[idx] = (int)((cur * 65L + 5) / 10);
      n++;
      int v = sts.ReadVoltage(id);
      if (!sts.getLastError() && v > 0) vSum += v;
      int ld = sts.ReadLoad(id);
      if (!sts.getLastError()) {
        int mag = ld & 0x3FF;
        if (mag > maxLoad) maxLoad = mag;
      }
    }
  }
  iMa = (sumRaw * 65L + 5) / 10;   // raw * 6.5 mA
  v10 = (n > 0) ? (vSum / n) : 0;  // deci-volts average
}

// Bus power snapshot for the TFT / web. Current LSB ≈ 6.5 mA.
static void cmdPwr() {
  int n = 0, v10 = 0, maxLoad = 0;
  long iMa = 0;
  gatherPowerStats(nullptr, n, iMa, v10, maxLoad);
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

static bool applySyncWrite(uint8_t n, uint8_t *ids, s16 *pos, u16 *spd,
                           u8 *acc) {
  dbgSyncWriteCalls++;
  if (n == 0 || n > MAX_N) {
    dbgSyncWriteFailures++;
    return false;
  }
  sts.SyncWritePosEx(ids, n, pos, spd, acc);
  return true;
}

static void doSyncWrite(uint8_t n, uint8_t *ids, s16 *pos, u16 *spd, u8 *acc) {
  if (applySyncWrite(n, ids, pos, spd, acc)) {
    replyOk();
  } else {
    replyErr();
  }
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
  if (strcmp(line, "DBG") == 0) {
    cmdDbg(false);
    return;
  }
  if (strcmp(line, "DBG RESET") == 0) {
    cmdDbg(true);
    return;
  }
  if (strncmp(line, "STREAM", 6) == 0) {
    int i = 6;
    long on = 0;
    if (parseInt(line, i, on)) {
      streaming = (on != 0);
      if (streaming) {
        // Prime every cache so the first cached replies are real data.
        streamFullPass();
        streamImuPass();
      }
    }
    Serial1.print(F("OK STREAM "));
    Serial1.println(streaming ? 1 : 0);
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
    // Per-joint current (mA) for logo glow, then paint status text.
    // Cache-served in STREAM mode (see gatherPowerStats).
    int ma[18];
    int n = 0, v10 = 0, maxLoad = 0;
    long iMa = 0;
    gatherPowerStats(ma, n, iMa, v10, maxLoad);
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

  dbgUnknownAscii++;
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
  dbgBinReplyHeaders++;
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

static int stsLe16(uint8_t lo, uint8_t hi) {
  return (int)((uint16_t)lo | ((uint16_t)hi << 8));
}

static int16_t stsSigned15(int w) {
  if (w & (1 << 15)) w = -(w & ~(1 << 15));
  return (int16_t)w;
}

static int stsSigned10(int w) {
  if (w & (1 << 10)) w = -(w & ~(1 << 10));
  return w;
}

// Decode 15-byte syncRead payload (addr 56..70). End=0 → lo,hi in Mem.
static void decodeFbPacket(const uint8_t *rx, int16_t &pos, int16_t &spd,
                           uint16_t &load, uint8_t &volt, uint8_t &temp,
                           uint8_t &mov, int16_t &cur) {
  pos = stsSigned15(stsLe16(rx[0], rx[1]));
  spd = stsSigned15(stsLe16(rx[2], rx[3]));
  int ld = stsSigned10(stsLe16(rx[4], rx[5]));
  load = (uint16_t)(ld < 0 ? -ld : ld);
  volt = rx[6];
  temp = rx[7];
  mov = rx[10];  // reg 66; rx[8..9] and rx[11..12] unused
  cur = stsSigned15(stsLe16(rx[13], rx[14]));
}

// ---- STREAM mode acquisition passes ----

// Fast pass: pos+speed for IDs 2..19 in one syncRead (~2-4 ms healthy).
static void streamFastPass() {
  uint32_t t0 = micros();
  dbgStreamFastPasses++;
  bool prevDefer = deferHostExec;
  deferHostExec = true;
  uint8_t ids[MAX_N];
  uint8_t n;
  fillDefaultIds(ids, n);
  ensureSyncRead(POS_SPD_MEM_LEN, streaming ? 5 : 20);
  sts.syncReadPacketTx(ids, n, SMS_STS_PRESENT_POSITION_L, POS_SPD_MEM_LEN);
  for (uint8_t k = 0; k < n; k++) {
    hostPump();  // host RX ring < one command frame — keep draining
    uint8_t rx[POS_SPD_MEM_LEN];
    if (sts.syncReadPacketRx(ids[k], rx)) {
      posCache[k] = stsSigned15(stsLe16(rx[0], rx[1]));
      spdCache[k] = stsSigned15(stsLe16(rx[2], rx[3]));
      posOk[k] = 1;
    } else {
      dbgStreamPosSlotFails++;
      posOk[k] = 0;
    }
  }
  hostPump();
  posSeq++;
  posStampMs = millis();
  deferHostExec = prevDefer;
  dbgUpdateMax(dbgMaxFastPassUs, (uint32_t)(micros() - t0));
}

// Full pass: 15-byte state block; refreshes the low-rate caches AND the
// fast caches (positions ride along for free).
static void streamFullPass() {
  uint32_t t0 = micros();
  dbgStreamFullPasses++;
  bool prevDefer = deferHostExec;
  deferHostExec = true;
  uint8_t ids[MAX_N];
  uint8_t n;
  fillDefaultIds(ids, n);
  ensureSyncRead(FB_MEM_LEN, streaming ? 8 : 20);
  sts.syncReadPacketTx(ids, n, SMS_STS_PRESENT_POSITION_L, FB_MEM_LEN);
  uint8_t failed[MAX_N];
  uint8_t nFailed = 0;
  for (uint8_t k = 0; k < n; k++) {
    hostPump();
    uint8_t rx[FB_MEM_LEN];
    if (sts.syncReadPacketRx(ids[k], rx)) {
      int16_t pos = 0, spd = 0, cur = 0;
      uint16_t load = 0;
      uint8_t volt = 0, temp = 0, mov = 0;
      decodeFbPacket(rx, pos, spd, load, volt, temp, mov, cur);
      posCache[k] = pos;
      spdCache[k] = spd;
      posOk[k] = 1;
      fbLoad[k] = load;
      fbVolt[k] = volt;
      fbTemp[k] = temp;
      fbMov[k] = mov;
      fbCur[k] = cur;
      fbOk[k] = 1;
    } else {
      dbgStreamFbSlotFails++;
      failed[nFailed++] = k;
    }
  }
  // Per-id FeedBack fallback for failed burst slots — same healing the
  // legacy cmdBulkFeedback path always had. Observed 2026-08-19: one
  // servo (ID 15) failed its slot in the 15-byte syncRead burst on
  // EVERY pass while answering direct reads perfectly, which made the
  // health watch report a healthy servo as missing. Direct reads are
  // the tiebreaker; only a servo that also fails these is really gone.
  for (uint8_t f = 0; f < nFailed; f++) {
    uint8_t k = failed[f];
    hostPump();
    int nLen = sts.FeedBack((int)ids[k]);
    if (nLen <= 0) nLen = sts.FeedBack((int)ids[k]);
    if (nLen > 0) {
      int16_t pos = 0, spd = 0, cur = 0;
      uint16_t load = 0;
      uint8_t volt = 0, temp = 0, mov = 0;
      decodeFbFromMem(pos, spd, load, volt, temp, mov, cur);
      posCache[k] = pos;
      spdCache[k] = spd;
      posOk[k] = 1;
      fbLoad[k] = load;
      fbVolt[k] = volt;
      fbTemp[k] = temp;
      fbMov[k] = mov;
      fbCur[k] = cur;
      fbOk[k] = 1;
    } else {
      dbgStreamFbSlotFails++;
      posOk[k] = 0;
      fbOk[k] = 0;
    }
  }
  hostPump();
  posSeq++;
  posStampMs = millis();
  fbStampMs = posStampMs;
  deferHostExec = prevDefer;
  dbgUpdateMax(dbgMaxFullPassUs, (uint32_t)(micros() - t0));
}

static void streamImuPassInner();

static void streamImuPass() {
  uint32_t t0 = micros();
  dbgStreamImuPasses++;
  bool prevDefer = deferHostExec;
  deferHostExec = true;
  hostPump();
  streamImuPassInner();
  deferHostExec = prevDefer;
  dbgUpdateMax(dbgMaxImuPassUs, (uint32_t)(micros() - t0));
}

static void streamImuPassInner() {
  if (!mpuReady) {
    unsigned long now = millis();
    if (now - imuRetryMs < 1000) return;  // don't hammer a dead sensor
    imuRetryMs = now;
    if (mpuEnsureReady() == 0) return;
  }
  uint8_t raw[14];
  if (!mpuReadRegs(MPU_REG_ACCEL_XOUT_H, raw, 14)) {
    mpuReady = false;
    return;
  }
  imuCache[0] = be16(raw + 0);
  imuCache[1] = be16(raw + 2);
  imuCache[2] = be16(raw + 4);
  imuCache[3] = be16(raw + 8);
  imuCache[4] = be16(raw + 10);
  imuCache[5] = be16(raw + 12);
  imuCache[6] = be16(raw + 6);  // TEMP_OUT
  imuCacheValid = true;
  imuStampMs = millis();
}

static void refreshSnapshotNow() {
  streamFastPass();
  streamImuPass();
}

static bool snapshotCacheFreshEnough() {
  if (!streaming || posStampMs == 0 || !imuCacheValid) return false;
  unsigned long now = millis();
  return (now - posStampMs <= HOST_S_MAX_CACHE_AGE_MS)
      && (now - imuStampMs <= HOST_S_MAX_CACHE_AGE_MS);
}

static void scheduleHostSnapshotRefresh() {
  unsigned long now = millis();
  hostSLastMs = now;
  hostSRefreshAtMs = now + HOST_S_REFRESH_DELAY_MS;
  hostSRefreshPending = true;
}

static void prepareSnapshotForHost() {
  dbgHostSnapshotRequests++;
  if (snapshotCacheFreshEnough()) {
    dbgHostSnapshotCacheHits++;
  } else {
    dbgHostSnapshotSyncRefreshes++;
    refreshSnapshotNow();
  }
  if (streaming) scheduleHostSnapshotRefresh();
}

static uint16_t ageMs(unsigned long stamp, bool valid) {
  if (!valid) return 0xFFFF;
  unsigned long d = millis() - stamp;
  return (d > 0xFFFEUL) ? 0xFFFE : (uint16_t)d;
}

// Snapshot reply for the 'S' combined command.
static void sendSnapshot() {
  uint8_t ids[MAX_N];
  uint8_t n;
  fillDefaultIds(ids, n);
  uint16_t posAge = ageMs(posStampMs, posStampMs != 0);
  uint16_t imuAge = ageMs(imuStampMs, imuCacheValid);
  uint8_t x = 0;
  sendBinHeader('s', n, x);
  sendBinByte((uint8_t)(posSeq & 0xFF), x);
  sendBinByte((uint8_t)(posSeq >> 8), x);
  sendBinByte((uint8_t)(posAge & 0xFF), x);
  sendBinByte((uint8_t)(posAge >> 8), x);
  sendBinByte((uint8_t)(imuAge & 0xFF), x);
  sendBinByte((uint8_t)(imuAge >> 8), x);
  for (uint8_t i = 0; i < 7; i++) {
    uint16_t v = (uint16_t)imuCache[i];
    sendBinByte((uint8_t)(v & 0xFF), x);
    sendBinByte((uint8_t)(v >> 8), x);
  }
  for (uint8_t k = 0; k < n; k++) {
    sendBinByte(ids[k], x);
    sendBinByte(posOk[k], x);
    uint16_t p = (uint16_t)posCache[k];
    sendBinByte((uint8_t)(p & 0xFF), x);
    sendBinByte((uint8_t)(p >> 8), x);
    uint16_t s = (uint16_t)spdCache[k];
    sendBinByte((uint8_t)(s & 0xFF), x);
    sendBinByte((uint8_t)(s >> 8), x);
  }
  Serial1.write(x);
}

// True when every requested id is in the streamed set (2..19).
static bool idsInStreamSet(uint8_t n, const uint8_t *ids) {
  for (uint8_t i = 0; i < n; i++) {
    if (ids[i] < ID_LO || ids[i] > ID_HI) return false;
  }
  return true;
}

// Full state: one syncRead TX for all IDs (15-byte block), FeedBack fallback.
// In STREAM mode this serves the caches instead (no servo-bus wait).
static void cmdBulkFeedback(uint8_t n, const uint8_t *ids) {
  uint8_t useN = n;
  uint8_t useIds[MAX_N];
  if (useN == 0) {
    fillDefaultIds(useIds, useN);
  } else {
    for (uint8_t i = 0; i < useN; i++) useIds[i] = ids[i];
  }

  if (streaming && fbStampMs != 0 && idsInStreamSet(useN, useIds)) {
    uint8_t x = 0;
    sendBinHeader('f', useN, x);
    for (uint8_t k = 0; k < useN; k++) {
      uint8_t idx = useIds[k] - ID_LO;
      uint8_t rec[13];
      rec[0] = useIds[k];
      rec[1] = (uint8_t)(posOk[idx] && fbOk[idx]);
      binPutI16(rec + 2, posCache[idx]);
      binPutI16(rec + 4, spdCache[idx]);
      binPutU16(rec + 6, fbLoad[idx]);
      rec[8] = fbVolt[idx];
      rec[9] = fbTemp[idx];
      rec[10] = fbMov[idx];
      binPutI16(rec + 11, fbCur[idx]);
      for (uint8_t i = 0; i < 13; i++) sendBinByte(rec[i], x);
    }
    Serial1.write(x);
    return;
  }

  bool usedSync = false;
  if (useN > 0) {
    ensureSyncRead(FB_MEM_LEN, 20);
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
// In STREAM mode this serves the position cache (no servo-bus wait).
static void cmdBulkPositions(uint8_t n, const uint8_t *ids) {
  uint8_t useN = n;
  uint8_t useIds[MAX_N];
  if (useN == 0) {
    fillDefaultIds(useIds, useN);
  } else {
    for (uint8_t i = 0; i < useN; i++) useIds[i] = ids[i];
  }

  if (streaming && posStampMs != 0 && idsInStreamSet(useN, useIds)) {
    uint8_t x = 0;
    sendBinHeader('p', useN, x);
    for (uint8_t k = 0; k < useN; k++) {
      uint8_t idx = useIds[k] - ID_LO;
      sendBinByte(useIds[k], x);
      sendBinByte(posOk[idx], x);
      uint16_t p = (uint16_t)posCache[idx];
      sendBinByte((uint8_t)(p & 0xFF), x);
      sendBinByte((uint8_t)(p >> 8), x);
    }
    Serial1.write(x);
    return;
  }

  bool usedSync = false;
  if (useN > 0) {
    ensureSyncRead(POS_MEM_LEN, 20);
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
  if (binCmd == 'S' && binN == 0) {
    // Snapshot-only query (no write).
    prepareSnapshotForHost();
    sendSnapshot();
    return;
  }
  if (binCmd == 'W' || binCmd == 'S') {
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
    if (binCmd == 'W') {
      doSyncWrite(binN, ids, pos, spd, acc);
      return;
    }
    // 'S' — combined step: apply the command first (lowest actuation
    // latency), then answer with the latest state snapshot. Without
    // streaming, refresh synchronously so the snapshot is still real.
    if (!applySyncWrite(binN, ids, pos, spd, acc)) {
      replyErr();
      return;
    }
    prepareSnapshotForHost();
    sendSnapshot();
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

static void execBinaryFrame() {
  uint32_t t0 = micros();
  dbgBinFramesExecuted++;
  dbgLastBinCmd = binCmd;
  dbgLastBinN = binN;
  handleBinaryFrame();
  dbgUpdateMax(dbgMaxBinExecUs, (uint32_t)(micros() - t0));
}

static void feedHostByte(uint8_t b) {
  // Binary frames A5 5A — otherwise ASCII lines.
  if (binState == 0) {
    if (b == 0xA5) {
      dbgBinFramesStarted++;
      dbgLastBinFrameStartUs = micros();
      binState = 1;
      return;
    }
    if (b == '\n') {
      dbgAsciiLinesCompleted++;
      lineBuf[lineLen] = '\0';
      if (deferHostExec) {
        dbgAsciiLinesParked++;
        parkedKind = 2;  // run after the pass (lineBuf/lineLen persist)
        return;
      }
      handleLine(lineBuf);
      lineLen = 0;
      return;
    }
    if (b == '\r') return;
    if (lineLen + 1 < sizeof(lineBuf)) {
      lineBuf[lineLen++] = (char)b;
    } else {
      dbgLineOverflow++;
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
    if (binCmd == 'W' || binCmd == 'S') {
      binNeed = (uint8_t)(binN * 6);
      if (binN == 0 || binN > MAX_N) binNeed = 0;
    } else if (binCmd == 'F' || binCmd == 'P') {
      // n==0 means "default IDs 2..19" — no id payload.
      binNeed = (binN > MAX_N) ? 0 : binN;
    } else {
      dbgBinBadCmd++;
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
    // checksum ('S' n=0 = snapshot-only query, no payload)
    bool okN = (binCmd == 'F' || binCmd == 'P' || binCmd == 'S')
                   ? (binN <= MAX_N && binGot == binNeed)
                   : (binN > 0 && binN <= MAX_N && binGot == binNeed);
    if (b == binXor && okN) {
      dbgBinFramesCompleted++;
      dbgLastBinFrameCompleteUs = micros();
      dbgLastBinCmd = binCmd;
      dbgLastBinN = binN;
      if (deferHostExec) {
        dbgBinFramesParked++;
        dbgBinParkedAtUs = dbgLastBinFrameCompleteUs;
        parkedKind = 1;  // binCmd/binN/binPayload persist until exec
      } else {
        execBinaryFrame();
      }
    } else {
      if (b != binXor) {
        dbgBinChecksumBad++;
      } else if (!okN) {
        dbgBinBadN++;
      }
      replyErr();  // TX only — safe mid-pass
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
  ensureSyncRead(FB_MEM_LEN, 20);
  Wire.begin();  // header SDA/SCL (D20/D21) → MPU-6050
  // MPU-6050 supports 400 kHz fast-mode I2C — the 14-byte sample read
  // drops from ~1.6 ms to ~0.5 ms, which matters at stream rates.
  Wire.setClock(400000);
}

static bool hostSeen = false;
static unsigned long lastHostMs = 0;
// Linux refreshes the panel every ~2 s; this much silence after first
// contact means the web service (or the SoC) died.
static const unsigned long HOST_LOST_MS = 12000;
// Once a binary host frame has started, the rest should arrive in ~1.2 ms
// at 921600 baud. Do not start another servo stream pass while the parser is
// mid-frame; if bytes really vanished, reset quickly and tell Linux.
static const unsigned long HOST_BIN_DESYNC_MS = 10;
// After this much silence, cut all servo torque so a dead brain can't
// leave motors fighting/cooking (2026-08-06 incident). Well above the
// ~15-20 s a normal `systemctl restart hexapod-web` gap takes.
static const unsigned long HOST_LIMP_MS = 30000;
static bool autoLimped = false;

// Drain host bytes into the frame parser (execution deferred — see
// parkedKind). Called between servo reads inside acquisition passes so
// the small host-UART RX ring can never overflow under a 113-byte
// 'W'/'S' frame.
static void hostPump() {
  while (parkedKind == 0 && Serial1.available() > 0) {
    dbgHostBytesSeen++;
    hostSeen = true;
    lastHostMs = millis();
    autoLimped = false;
    feedHostByte((uint8_t)Serial1.read());
  }
}

static void execParked() {
  uint8_t kind = parkedKind;
  parkedKind = 0;
  if (kind == 1) {
    if (dbgBinParkedAtUs != 0) {
      dbgUpdateMax(
          dbgMaxParkedWaitUs, (uint32_t)(micros() - dbgBinParkedAtUs));
      dbgBinParkedAtUs = 0;
    }
    execBinaryFrame();
  } else if (kind == 2) {
    handleLine(lineBuf);
    lineLen = 0;
  }
}

void loop() {
  unsigned long now = millis();
  // A frame parked during the synchronous (non-streaming) 'S' passes
  // must still run; parked work always precedes new ring bytes.
  if (parkedKind != 0) execParked();
  // Desync guard: a torn binary frame (host retry after timeout) must
  // not eat the next frame's header as payload.
  if (binState != 0 && now - lastHostMs > HOST_BIN_DESYNC_MS) {
    dbgDesyncResets++;
    binState = 0;
    replyErr();
  }
  if (Serial1.available() > 0) {
    hostSeen = true;
    lastHostMs = millis();
    autoLimped = false;
    do {
      dbgHostBytesSeen++;
      feedHostByte((uint8_t)Serial1.read());
    } while (Serial1.available() > 0);
    return;
  }
  // A frame is in progress but the next byte has not arrived yet. Spin here
  // instead of entering a servo/I2C stream pass; otherwise the small hardware
  // FIFO can overflow before hostPump() gets another chance to drain it.
  if (binState != 0) return;
  if (streaming && hostSRefreshPending
      && (long)(now - hostSRefreshAtMs) >= 0) {
    hostSRefreshPending = false;
    refreshSnapshotNow();
    dbgHostSnapshotAsyncRefreshes++;
    if (parkedKind != 0) {
      execParked();
      return;
    }
    return;
  }
  if (streaming && hostSLastMs != 0
      && (long)(now - hostSLastMs) < HOST_S_CONTROL_IDLE_MS) {
    return;
  }
  if (streaming) {
    // Free-running acquisition while the host line is idle. ONE pass per
    // loop() iteration keeps worst-case host-command latency to a single
    // pass (~3-8 ms). Host bytes arriving mid-pass are pumped into the
    // parser (hostPump) and a completed frame runs right after the pass.
    if (now - fbStampMs >= FB_PERIOD_MS) {
      streamFullPass();   // low-rate: current/load/volt/temp (~10 Hz)
    } else {
      streamFastPass();   // pos+speed, all 18 servos (~150-250 Hz)
      streamImuPass();
    }
    if (parkedKind != 0) {
      execParked();
      return;
    }
  }
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
