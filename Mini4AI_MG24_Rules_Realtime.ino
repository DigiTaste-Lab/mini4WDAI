// ============================================================
// Mini4AI MG24 Rules Realtime Firmware
// v3.6: explicit rule priority + curve hold segments (LONG_PEAK removed), no LDA, BLE-free runtime control
// XIAO MG24 Sense + LSM6DS3 + BLE
//
// Production policy:
//   - No LDA
//   - No realtime gyro/features notify
//   - No Web-side decision making during run
//   - 200 Hz local IMU -> rule -> PWM control
//   - BLE is only config/start/stop + stopped log dump
//   - Optional low-rate segment event notify for debugging only
//
// BLE service/major UUIDs are kept compatible with the previous app
// where practical, so the Web UI can be swapped without changing the
// board identity.
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <math.h>
#include <string.h>

// ============================================================
// Hardware
// ============================================================
#define DEVICE_NAME "MiniYonAI_01"

static const uint8_t PWM_PIN    = PC0;
static const uint8_t IMU_EN_PIN = PD5;

#ifndef LED_BUILTIN
#define LED_BUILTIN LED_RED
#endif

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// ============================================================
// Control constants
// ============================================================
static const uint32_t CONTROL_HZ      = 200;
static const uint32_t CONTROL_DT_MS   = 1000 / CONTROL_HZ;  // 5 ms
static const uint32_t BLE_POLL_MS     = 20;                 // 50 Hz max
static const uint32_t LIVE_EVENT_MS   = 150;                // optional debug only
static const uint16_t GZ_BIAS_SAMPLES = 80;
static const uint8_t  GZ_FILT_N       = 3;

// Segment IDs. Kept intentionally simple.
static const uint8_t SEG_NONE     = 0;
static const uint8_t SEG_STRAIGHT = 1;
static const uint8_t SEG_RIGHT    = 2;  // gz negative in this firmware
static const uint8_t SEG_LEFT     = 4;  // gz positive in this firmware
static const uint8_t SEG_WAVE     = 6;
static const uint8_t SEG_LEFT_PEAK       = 7;  // emitted after LEFT curve peak passed
static const uint8_t SEG_RIGHT_PEAK      = 8;  // emitted after RIGHT curve peak passed
static const uint8_t SEG_STRAIGHT_HOLD   = 9;  // emitted after straight continues for configured time
static const uint8_t SEG_LEFT_HOLD        = 10; // emitted when a LEFT curve continues for configured time
static const uint8_t SEG_RIGHT_HOLD       = 11; // emitted when a RIGHT curve continues for configured time

// Actions
static const uint8_t ACTION_NONE  = 0;
static const uint8_t ACTION_BRAKE = 1;
static const uint8_t ACTION_SPEED = 2;
static const uint8_t ACTION_STOP  = 4;
static const uint8_t ACTION_WAIT  = 5;

// Rule trigger modes. The match counter is per rule and increments every time
// the pattern is recognized, regardless of whether actions fire.
static const uint8_t TRIG_EVERY   = 0;
static const uint8_t TRIG_ODD     = 1;
static const uint8_t TRIG_EVEN    = 2;
static const uint8_t TRIG_EVERY_N = 3;
static const uint8_t TRIG_ONLY_N  = 4;

// Commands
static const uint8_t CMD_STOP           = 0;
static const uint8_t CMD_ARM            = 1;  // legacy: reset/ready only. UI v3.1 does not expose this.
static const uint8_t CMD_START          = 2;
static const uint8_t CMD_LOG_REQUEST    = 4;
static const uint8_t CMD_CLEAR_LOG      = 5;

// Status
static const uint8_t STATUS_IDLE    = 0;
static const uint8_t STATUS_READY   = 1;   // legacy ARM value, UI no longer exposes ARM
static const uint8_t STATUS_RUNNING = 2;
static const uint8_t STATUS_STOPPED = 3;
static const uint8_t STATUS_ERROR   = 4;

// Rule packet limits
static const uint8_t MAX_PATTERN_LEN = 8;
static const uint8_t MAX_RULES       = 8;
static const uint8_t MAX_ACTIONS     = 5;
// v37: Rule packet expanded to 42 bytes. Layout:
//   q[0]                  patternLen
//   q[1..8]               pattern[8]
//   q[9]                  actionCount
//   q[10..14]             action types[5]
//   q[15..24]             action durations u16 LE [5]
//   q[25..29]             action strengths[5]
//   q[30]                 loopMode
//   q[31]                 triggerMode
//   q[32]                 triggerN
//   q[33]                 priority (0=legacy auto)
//   q[34..35]             peakAfterOverrideMs    (0xFFFF=use global)
//   q[36..37]             straightHoldOverrideMs (0xFFFF=use global)
//   q[38..39]             curveLongOverrideMs    (0xFFFF=use global)
//   q[40..41]             reserved
// Header byte buf[0] = protocol version: 2 (v36 legacy 36B) | 3 (v37 42B).
static const uint8_t RULE_BYTES      = 42;
static const uint8_t RULE_BYTES_V2   = 36;  // legacy v36 packet, still accepted

// ============================================================
// Tunable parameters, writable from params characteristic
// Packet: 30 bytes, LE
// [0..1]   cornerTh dps
// [2..3]   straightTh dps
// [4..5]   wavePosTh dps
// [6..7]   waveNegTh dps
// [8..9]   wavePairWindow ms
// [10..11] waveLatch ms
// [12..13] waveCooldown ms
// [14..15] exitDebounce ms
// [16]     normalDuty
// [17]     recoverDuty
// [18]     startDuty
// [19]     flags: bit0 enable low-rate live segment notify
// [20..21] curvePeakAfterMs: peak must be older than this
// [22..23] curvePeakDropDps: emit peak after abs(gz) drops this much from peak
// [24..25] straightHoldMs: emit STRAIGHT_HOLD after this duration below STRAIGHT_TH
// [26..27] segmentCooldownMs: suppress duplicate same segment events inside this window
// [28..29] curveLongMs: curve must continue this long to emit LEFT/RIGHT_HOLD
// ============================================================
static float    CORNER_TH       = 260.0f;
static float    STRAIGHT_TH     = 220.0f;
static float    WAVE_POS_TH     = 200.0f;
static float    WAVE_NEG_TH     = 200.0f;
static uint32_t WAVE_WINDOW_MS  = 200;
static uint32_t WAVE_LATCH_MS   = 1200;
static uint32_t WAVE_COOLDOWN_MS= 400;
static uint32_t EXIT_DEBOUNCE_MS= 15;
static uint8_t  NORMAL_DUTY     = 255;
static uint8_t  RECOVER_DUTY    = 100;
static uint8_t  START_DUTY      = 255;
static bool     DEBUG_LIVE_SEG  = false;
static uint32_t CURVE_PEAK_AFTER_MS = 30;
static float    CURVE_PEAK_DROP_DPS = 40.0f;
static uint32_t STRAIGHT_HOLD_MS    = 250;
static uint32_t SEGMENT_COOLDOWN_MS = 80;
static uint32_t CURVE_LONG_MS        = 180;

// Soft-start is intentionally short. Set START_DUTY=NORMAL_DUTY for instant start.
static const uint32_t SOFT_START_MS = 120;

// ============================================================
// Rule model
// ============================================================
struct RuleAction {
  uint8_t  type;
  uint16_t durationMs;
  uint8_t  strength;    // for SPEED: duty 0..255. If 1..4, maps to 25/50/75/100%.
};

struct Rule {
  uint8_t pattern[MAX_PATTERN_LEN];
  uint8_t patternLen;
  RuleAction actions[MAX_ACTIONS];
  uint8_t actionCount;
  bool loopMode;
  uint8_t matchIdx;
  uint8_t triggerMode;
  uint8_t triggerN;
  uint8_t priority;     // 1..255. Higher value wins. 0 in packet means legacy auto-priority.
  uint16_t matchCount;
  bool enabled;
  // v37 per-rule timing overrides. 0xFFFF = inherit global.
  uint16_t peakAfterOverrideMs;
  uint16_t straightHoldOverrideMs;
  uint16_t curveLongOverrideMs;
  // Pending-fire state. When the final pattern segment requires more time than
  // the global emit threshold (override > global), the rule waits here until
  // its own threshold is satisfied, then fires.
  bool     pending;
  uint32_t pendingFireDueMs;   // absolute millis() when pending becomes ready
  float    pendingGz;          // gz captured at pending start, used for action log
  uint8_t  pendingSeg;         // segment that triggered the pending state
};

static Rule rules[MAX_RULES];
static uint8_t rulesCount = 0;

// ============================================================
// Runtime state
// ============================================================
static bool bleReady = false;
static bool bleConnected = false;
static bool isRunning = false;
static uint8_t statusValue = STATUS_IDLE;

static uint32_t runStartMs = 0;
static uint32_t lastControlMs = 0;
static uint32_t lastBlePollMs = 0;
static uint32_t lastLiveEventMs = 0;

static float gzBias = 0.0f;
static float gzBuf[GZ_FILT_N] = {0};
static uint8_t gzIdx = 0;

static bool inCorner = false;
static int8_t cornerSign = 0;     // +1 left, -1 right
static uint32_t cornerStartMs = 0;
static uint32_t straightCandMs = 0;
static uint32_t straightHoldStartMs = 0;
static bool straightHoldEmitted = false;
static float cornerPeakAbs = 0.0f;
static uint32_t cornerPeakMs = 0;
static bool cornerPeakEmitted = false;
static bool cornerHoldEmitted = false;
static uint32_t lastSegEventMs[16] = {0};
static uint32_t lastPosSpikeMs = 0;
static uint32_t lastNegSpikeMs = 0;
static uint32_t waveLatchUntilMs = 0;
static uint32_t waveCooldownUntilMs = 0;
static uint8_t lastSegment = SEG_NONE;

// Action queue execution
static RuleAction activeActions[MAX_ACTIONS];
static uint8_t activeActionCount = 0;
static uint8_t activeActionIdx = 0;
static bool actionActive = false;
static uint32_t actionEndMs = 0;
static uint8_t activeRuleIdx = 0xFF;
static uint8_t activePriority = 0;
static float activeTriggerGz = 0.0f;
static uint8_t currentMotorDuty = 0;

// Optional low-rate live event notify queue
static volatile bool pendingLiveSeg = false;
static uint8_t pendingLiveSegValue = SEG_NONE;

// ============================================================
// Event log. Only section/action events are logged during run.
// No 200 Hz BLE transfer during run.
// ============================================================
struct EventLog {
  uint16_t tMs;       // elapsed from runStartMs, saturated
  uint8_t  seg;
  uint8_t  ruleIdx;   // 0xFF if none
  uint8_t  action;
  int16_t  gz10;
};

static const uint16_t LOG_CAP = 384;
static EventLog logBuf[LOG_CAP];
static uint16_t logCount = 0;
static bool dumpActive = false;
static uint16_t dumpIndex = 0;
static uint16_t dumpSeq = 0;

// ============================================================
// BLE service and characteristics
// ============================================================
BLEService configService("12345678-1234-5678-1234-56789abcdef0");

BLECharacteristic commandChar(
  "12345678-1234-5678-1234-56789abcdef4",
  BLERead | BLEWrite,
  1
);

BLEByteCharacteristic statusChar(
  "12345678-1234-5678-1234-56789abcdef5",
  BLERead | BLENotify
);

BLEByteCharacteristic currentSegChar(
  "12345678-1234-5678-1234-56789abcdef6",
  BLERead | BLENotify
);

BLEByteCharacteristic speedChar(
  "12345678-1234-5678-1234-56789abcdefa",
  BLERead | BLEWrite
);

// Reuses the old thresholds UUID as production params.
BLECharacteristic paramsChar(
  "12345678-1234-5678-1234-56789abcdf00",
  BLERead | BLEWrite,
  30
);

BLECharacteristic rulesChar(
  "12345678-1234-5678-1234-56789abcdefd",
  BLERead | BLEWrite,
  300
);

// Stopped log dump. Packet: [seqLo seqHi count events...]
// event = [tLo tHi seg rule action gzLo gzHi], up to 2 events/packet.
// terminator: seq=0xFFFF, count=0.
BLECharacteristic bulkChar(
  "12345678-1234-5678-1234-56789abcdefc",
  BLERead | BLENotify,
  20
);

// ============================================================
// Small utilities
// ============================================================
static inline bool reached(uint32_t now, uint32_t t) {
  return (int32_t)(now - t) >= 0;
}

static uint16_t readU16LE(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void writeU16LE(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

static int16_t clampI16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

static uint8_t clampU8(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

static uint8_t dutyFromStrength(uint8_t s) {
  if (s == 0) return 0;
  if (s == 1) return 64;
  if (s == 2) return 128;
  if (s == 3) return 192;
  if (s == 4) return 255;
  return s;
}

static void motorDuty(uint8_t d) {
  currentMotorDuty = d;
  analogWrite(PWM_PIN, d);
}

static void motorBrake() {
  motorDuty(0);
}

static void ledSet(bool on) {
  digitalWrite(LED_BUILTIN, on ? LOW : HIGH);  // many XIAO LEDs are active-low
}

static void setStatus(uint8_t s) {
  if (statusValue == s) return;
  statusValue = s;
  if (bleReady) statusChar.writeValue(statusValue);
}

static void writeParamsToChar() {
  uint8_t p[30] = {0};
  writeU16LE(&p[0],  (uint16_t)lroundf(CORNER_TH));
  writeU16LE(&p[2],  (uint16_t)lroundf(STRAIGHT_TH));
  writeU16LE(&p[4],  (uint16_t)lroundf(WAVE_POS_TH));
  writeU16LE(&p[6],  (uint16_t)lroundf(WAVE_NEG_TH));
  writeU16LE(&p[8],  (uint16_t)WAVE_WINDOW_MS);
  writeU16LE(&p[10], (uint16_t)WAVE_LATCH_MS);
  writeU16LE(&p[12], (uint16_t)WAVE_COOLDOWN_MS);
  writeU16LE(&p[14], (uint16_t)EXIT_DEBOUNCE_MS);
  p[16] = NORMAL_DUTY;
  p[17] = RECOVER_DUTY;
  p[18] = START_DUTY;
  p[19] = DEBUG_LIVE_SEG ? 1 : 0;
  writeU16LE(&p[20], (uint16_t)CURVE_PEAK_AFTER_MS);
  writeU16LE(&p[22], (uint16_t)lroundf(CURVE_PEAK_DROP_DPS));
  writeU16LE(&p[24], (uint16_t)STRAIGHT_HOLD_MS);
  writeU16LE(&p[26], (uint16_t)SEGMENT_COOLDOWN_MS);
  writeU16LE(&p[28], (uint16_t)CURVE_LONG_MS);
  paramsChar.writeValue(p, sizeof(p));
  speedChar.writeValue(NORMAL_DUTY);
}

static void logReset() {
  logCount = 0;
  dumpActive = false;
  dumpIndex = 0;
  dumpSeq = 0;
}

static void logEvent(uint8_t seg, uint8_t ruleIdx, uint8_t action, float gz) {
  if (logCount >= LOG_CAP) return;
  uint32_t dt = millis() - runStartMs;
  if (dt > 65535UL) dt = 65535UL;
  logBuf[logCount].tMs = (uint16_t)dt;
  logBuf[logCount].seg = seg;
  logBuf[logCount].ruleIdx = ruleIdx;
  logBuf[logCount].action = action;
  logBuf[logCount].gz10 = clampI16(gz * 10.0f);
  logCount++;
}

static void dumpStart() {
  if (!bleReady) return;
  dumpActive = true;
  dumpIndex = 0;
  dumpSeq = 0;
}

static void dumpStep() {
  if (!dumpActive || !bleReady || isRunning) return;

  uint8_t pkt[20] = {0};
  if (dumpIndex >= logCount) {
    pkt[0] = 0xFF;
    pkt[1] = 0xFF;
    pkt[2] = 0;
    bulkChar.writeValue(pkt, 3);
    dumpActive = false;
    return;
  }

  writeU16LE(&pkt[0], dumpSeq++);
  uint8_t count = 0;
  uint8_t off = 3;
  while (count < 2 && dumpIndex < logCount) {
    const EventLog &e = logBuf[dumpIndex++];
    writeU16LE(&pkt[off + 0], e.tMs);
    pkt[off + 2] = e.seg;
    pkt[off + 3] = e.ruleIdx;
    pkt[off + 4] = e.action;
    writeU16LE(&pkt[off + 5], (uint16_t)e.gz10);
    off += 7;
    count++;
  }
  pkt[2] = count;
  bulkChar.writeValue(pkt, off);
}

// ============================================================
// IMU
// ============================================================
static float calibrateGzBias() {
  float sum = 0.0f;
  for (uint16_t i = 0; i < GZ_BIAS_SAMPLES; i++) {
    sum += myIMU.readFloatGyroZ();
    delay(2);
  }
  return sum / (float)GZ_BIAS_SAMPLES;
}

static float updateGzFilter(float gz) {
  gzBuf[gzIdx] = gz;
  gzIdx = (uint8_t)((gzIdx + 1) % GZ_FILT_N);
  float sum = 0.0f;
  for (uint8_t i = 0; i < GZ_FILT_N; i++) sum += gzBuf[i];
  return sum / (float)GZ_FILT_N;
}

// ============================================================
// Rule/action execution
// ============================================================
static void startActionsFromRule(uint8_t ruleIdx, float gz, uint32_t now);

static uint8_t priorityForRule(uint8_t ruleIdx) {
  if (ruleIdx >= rulesCount) return 0;
  uint8_t p = rules[ruleIdx].priority;
  if (p == 0) {
    // Legacy fallback: earlier rules are slightly higher priority.
    return (uint8_t)(100 - ruleIdx);
  }
  return p;
}

static bool segmentMatches(uint8_t expected, uint8_t actual) {
  if (expected == SEG_NONE) return false;
  return expected == actual;
}

static bool ruleContainsSegment(const Rule &r, uint8_t seg) {
  for (uint8_t i = 0; i < r.patternLen; i++) {
    if (r.pattern[i] == seg) return true;
  }
  return false;
}

static bool isAuxSegment(uint8_t seg) {
  return seg == SEG_WAVE || seg == SEG_LEFT_PEAK || seg == SEG_RIGHT_PEAK || seg == SEG_STRAIGHT_HOLD || seg == SEG_LEFT_HOLD || seg == SEG_RIGHT_HOLD;
}

// ---- v37 per-rule override helpers ----
//
// Compute the *effective* emit threshold for a given segment kind, taking the
// minimum of the global value and any per-rule override that targets this kind
// of segment. This way, if any rule wants to fire earlier than the global, the
// firmware emits earlier; rules that want to fire LATER use the pending state
// (handled in advanceRuleWithSegment + updatePendingFires).
static bool segWantsPeakAfter(uint8_t seg) {
  return seg == SEG_LEFT_PEAK || seg == SEG_RIGHT_PEAK;
}
static bool segWantsStraightHold(uint8_t seg) {
  return seg == SEG_STRAIGHT_HOLD;
}
static bool segWantsCurveLong(uint8_t seg) {
  return seg == SEG_LEFT_HOLD || seg == SEG_RIGHT_HOLD;
}

// Smallest override value among rules whose pattern contains 'seg'. Returns
// 0xFFFF if no rule has an override for this segment kind.
static uint16_t minOverrideFor(uint8_t seg) {
  uint16_t best = 0xFFFF;
  for (uint8_t i = 0; i < rulesCount; i++) {
    const Rule &r = rules[i];
    if (!r.enabled || r.patternLen == 0) continue;
    bool hasSeg = false;
    for (uint8_t k = 0; k < r.patternLen; k++) if (r.pattern[k] == seg) { hasSeg = true; break; }
    if (!hasSeg) continue;
    uint16_t v = 0xFFFF;
    if (segWantsPeakAfter(seg))           v = r.peakAfterOverrideMs;
    else if (segWantsStraightHold(seg))   v = r.straightHoldOverrideMs;
    else if (segWantsCurveLong(seg))      v = r.curveLongOverrideMs;
    if (v != 0xFFFF && v < best) best = v;
  }
  return best;
}

static uint16_t effectivePeakAfterMs() {
  uint16_t override_min = 0xFFFF;
  uint16_t a = minOverrideFor(SEG_LEFT_PEAK);       if (a < override_min) override_min = a;
  uint16_t b = minOverrideFor(SEG_RIGHT_PEAK);      if (b < override_min) override_min = b;
  uint32_t g = CURVE_PEAK_AFTER_MS;
  return (uint16_t)((override_min < g) ? override_min : g);
}
static uint16_t effectiveStraightHoldMs() {
  uint16_t o = minOverrideFor(SEG_STRAIGHT_HOLD);
  uint32_t g = STRAIGHT_HOLD_MS;
  return (uint16_t)((o < g) ? o : g);
}
static uint16_t effectiveCurveLongMs() {
  uint16_t override_min = 0xFFFF;
  uint16_t a = minOverrideFor(SEG_LEFT_HOLD);       if (a < override_min) override_min = a;
  uint16_t b = minOverrideFor(SEG_RIGHT_HOLD);      if (b < override_min) override_min = b;
  uint32_t g = CURVE_LONG_MS;
  return (uint16_t)((override_min < g) ? override_min : g);
}

// True if the rule's own override (if any) for this segment kind has been
// satisfied by the elapsed time since the relevant detection start.
static bool ruleOverrideSatisfied(const Rule &r, uint8_t seg, uint32_t now) {
  uint16_t ov = 0xFFFF;
  uint32_t startMs = 0;
  if (segWantsPeakAfter(seg)) {
    ov = r.peakAfterOverrideMs;
    startMs = cornerPeakMs;
  } else if (segWantsStraightHold(seg)) {
    ov = r.straightHoldOverrideMs;
    startMs = straightHoldStartMs ? straightHoldStartMs : now;
  } else if (segWantsCurveLong(seg)) {
    ov = r.curveLongOverrideMs;
    startMs = cornerStartMs;
  } else {
    return true;  // not a timed segment
  }
  if (ov == 0xFFFF) return true;  // no override, accept emit immediately
  return (uint32_t)(now - startMs) >= (uint32_t)ov;
}

static bool ruleTriggerAllowsFire(Rule &r) {
  r.matchCount++;
  uint8_t n = r.triggerN == 0 ? 1 : r.triggerN;
  switch (r.triggerMode) {
    case TRIG_ODD:     return (r.matchCount % 2) == 1;
    case TRIG_EVEN:    return (r.matchCount % 2) == 0;
    case TRIG_EVERY_N: return (r.matchCount % n) == 0;
    case TRIG_ONLY_N:  return r.matchCount == n;
    case TRIG_EVERY:
    default:           return true;
  }
}

// Production matcher policy:
//   - Exact order is kept.
//   - Duplicate events of the last matched segment are ignored.
//   - WAVE is ignored while the rule is waiting for LEFT/RIGHT.
// This is intentional: on a real car, wave oscillation and corner vibration can
// insert WAVE/RIGHT/LEFT noise between structural course sections. The motor
// action must not depend on a fragile exact event stream.
static void advanceRuleWithSegment(Rule &r, uint8_t seg, uint8_t ruleIdx, float gz, uint32_t now) {
  if (!r.enabled || r.patternLen == 0) return;

  uint8_t expected = r.pattern[r.matchIdx];

  if (segmentMatches(expected, seg)) {
    r.matchIdx++;
    if (r.matchIdx >= r.patternLen) {
      r.matchIdx = 0;
      // v37: per-rule override timing. If the rule's own threshold is not yet
      // satisfied, defer to pending state and let updatePendingFires() retry.
      if (!ruleOverrideSatisfied(r, seg, now)) {
        uint16_t ov = 0xFFFF; uint32_t startMs = now;
        if (segWantsPeakAfter(seg))         { ov = r.peakAfterOverrideMs;    startMs = cornerPeakMs; }
        else if (segWantsStraightHold(seg)) { ov = r.straightHoldOverrideMs; startMs = straightHoldStartMs ? straightHoldStartMs : now; }
        else if (segWantsCurveLong(seg))    { ov = r.curveLongOverrideMs;    startMs = cornerStartMs; }
        if (ov != 0xFFFF) {
          r.pending = true;
          r.pendingFireDueMs = startMs + (uint32_t)ov;
          r.pendingGz = gz;
          r.pendingSeg = seg;
          return;
        }
      }
      if (ruleTriggerAllowsFire(r)) {
        startActionsFromRule(ruleIdx, gz, now);
        if (!r.loopMode) r.enabled = false;
      } else {
        logEvent(seg, ruleIdx, ACTION_NONE, gz);
      }
    }
    return;
  }

  if (r.matchIdx > 0 && segmentMatches(r.pattern[r.matchIdx - 1], seg)) {
    return;
  }
  if (isAuxSegment(seg) && expected != seg) {
    return;
  }
  r.matchIdx = segmentMatches(r.pattern[0], seg) ? 1 : 0;
}

// Process rules that completed their pattern but were waiting for their own
// per-rule timing override to be satisfied.
static void updatePendingFires(uint32_t now) {
  for (uint8_t i = 0; i < rulesCount; i++) {
    Rule &r = rules[i];
    if (!r.pending) continue;
    if ((int32_t)(now - r.pendingFireDueMs) < 0) continue;
    r.pending = false;
    if (ruleTriggerAllowsFire(r)) {
      startActionsFromRule(i, r.pendingGz, now);
      if (!r.loopMode) r.enabled = false;
    } else {
      logEvent(r.pendingSeg, i, ACTION_NONE, r.pendingGz);
    }
  }
}

static void resetRuleProgress() {
  for (uint8_t i = 0; i < MAX_RULES; i++) {
    rules[i].matchIdx = 0;
    rules[i].matchCount = 0;
    rules[i].enabled = true;
    rules[i].pending = false;
  }
  activeRuleIdx = 0xFF;
  activePriority = 0;
}

static void startNextAction(uint32_t now) {
  while (activeActionIdx < activeActionCount) {
    RuleAction &a = activeActions[activeActionIdx];

    if (a.type == ACTION_NONE) {
      activeActionIdx++;
      continue;
    }

    // Log every action start. This makes stopped logs directly show whether
    // BRAKE actually fired, instead of only showing the segment events.
    logEvent(lastSegment, activeRuleIdx, a.type, activeTriggerGz);

    if (a.type == ACTION_STOP) {
      motorBrake();
      isRunning = false;
      actionActive = false;
      setStatus(STATUS_STOPPED);
      dumpStart();
      return;
    }

    if (a.type == ACTION_BRAKE) {
      motorBrake();
    } else if (a.type == ACTION_SPEED) {
      motorDuty(dutyFromStrength(a.strength));
    } else if (a.type == ACTION_WAIT) {
      motorDuty(NORMAL_DUTY);
    }

    uint16_t dur = a.durationMs;
    if (dur == 0) dur = 1;
    actionEndMs = now + dur;
    actionActive = true;
    return;
  }

  actionActive = false;
  activeActionCount = 0;
  activeActionIdx = 0;
  activeRuleIdx = 0xFF;
  activePriority = 0;
  activeTriggerGz = 0.0f;
  if (isRunning) motorDuty(NORMAL_DUTY);
}

static void startActionsFromRule(uint8_t ruleIdx, float gz, uint32_t now) {
  if (ruleIdx >= rulesCount) return;
  Rule &r = rules[ruleIdx];
  if (r.actionCount == 0) return;

  uint8_t prio = priorityForRule(ruleIdx);
  // Higher priority can preempt lower-priority actions.
  // If priority is equal, the earlier rule index wins.
  if (actionActive) {
    if (prio < activePriority) return;
    if (prio == activePriority && activeRuleIdx != 0xFF && ruleIdx >= activeRuleIdx) return;
  }

  for (uint8_t i = 0; i < MAX_ACTIONS; i++) {
    activeActions[i] = r.actions[i];
  }
  activeActionCount = r.actionCount;
  activeActionIdx = 0;
  activeRuleIdx = ruleIdx;
  activePriority = prio;
  activeTriggerGz = gz;
  startNextAction(now);
}

static void updateActionEngine(uint32_t now) {
  if (!actionActive) return;
  if (reached(now, actionEndMs)) {
    activeActionIdx++;
    startNextAction(now);
  }
}

static void onSegmentDetected(uint8_t seg, float gz, uint32_t now) {
  if (seg < 16 && lastSegEventMs[seg] != 0 && (uint32_t)(now - lastSegEventMs[seg]) < SEGMENT_COOLDOWN_MS) {
    return;
  }
  if (seg < 16) lastSegEventMs[seg] = now;

  lastSegment = seg;
  logEvent(seg, 0xFF, ACTION_NONE, gz);

  if (DEBUG_LIVE_SEG) {
    pendingLiveSegValue = seg;
    pendingLiveSeg = true;
  }

  // Evaluate rules by explicit priority, not by visual list order alone.
  // This prevents a broad low-priority rule from consuming a segment before
  // a narrow high-priority safety rule can fire. Same priority falls back to
  // the list order: earlier rule wins.
  bool used[MAX_RULES] = {false};
  for (uint8_t pass = 0; pass < rulesCount; pass++) {
    int best = -1;
    uint8_t bestPrio = 0;
    for (uint8_t i = 0; i < rulesCount; i++) {
      if (used[i]) continue;
      uint8_t p = priorityForRule(i);
      if (best < 0 || p > bestPrio || (p == bestPrio && i < (uint8_t)best)) {
        best = i;
        bestPrio = p;
      }
    }
    if (best < 0) break;
    used[best] = true;
    advanceRuleWithSegment(rules[best], seg, (uint8_t)best, gz, now);
  }
}

// ============================================================
// Segment detection. Emits curve-start events, not curve-end events,
// so rules can fire quickly like the M5StampS3 version.
// ============================================================
static void resetDetection(uint32_t now) {
  inCorner = false;
  cornerSign = 0;
  cornerStartMs = now;
  straightCandMs = 0;
  straightHoldStartMs = 0;
  straightHoldEmitted = false;
  cornerPeakAbs = 0.0f;
  cornerPeakMs = now;
  cornerPeakEmitted = false;
  cornerHoldEmitted = false;
  for (uint8_t i = 0; i < 16; i++) lastSegEventMs[i] = 0;
  lastPosSpikeMs = 0;
  lastNegSpikeMs = 0;
  waveLatchUntilMs = 0;
  waveCooldownUntilMs = 0;
  lastSegment = SEG_NONE;
  for (uint8_t i = 0; i < GZ_FILT_N; i++) gzBuf[i] = 0.0f;
  gzIdx = 0;
}

static void updateSegmentDetection(float gz, uint32_t now) {
  float a = fabsf(gz);

  // Wave: M5-like positive/negative spike pair within a time window.
  // If the car naturally reports wave as LEFT->RIGHT->LEFT, rules can simply
  // use those structural segments and ignore SEG_WAVE.
  if (gz > WAVE_POS_TH) lastPosSpikeMs = now;
  if (gz < -WAVE_NEG_TH) lastNegSpikeMs = now;

  if (reached(now, waveCooldownUntilMs) && lastPosSpikeMs > 0 && lastNegSpikeMs > 0) {
    uint32_t gap = (lastPosSpikeMs > lastNegSpikeMs)
                 ? (lastPosSpikeMs - lastNegSpikeMs)
                 : (lastNegSpikeMs - lastPosSpikeMs);
    if (gap > 0 && gap <= WAVE_WINDOW_MS) {
      waveLatchUntilMs = now + WAVE_LATCH_MS;
      waveCooldownUntilMs = now + WAVE_COOLDOWN_MS;
      lastPosSpikeMs = 0;
      lastNegSpikeMs = 0;
      onSegmentDetected(SEG_WAVE, gz, now);
      // Do not return. A strong wave sample can also be the start of a curve.
    }
  }

  // Corner start: emit immediately for fast rule matching.
  if (!inCorner && a > CORNER_TH) {
    inCorner = true;
    cornerStartMs = now;
    straightCandMs = 0;
    straightHoldStartMs = 0;
    straightHoldEmitted = false;
    cornerSign = (gz >= 0.0f) ? +1 : -1;
    cornerPeakAbs = a;
    cornerPeakMs = now;
    cornerPeakEmitted = false;
    cornerHoldEmitted = false;
    uint8_t seg = (cornerSign > 0) ? SEG_LEFT : SEG_RIGHT;
    onSegmentDetected(seg, gz, now);
    return;
  }

  if (inCorner) {
    // Track the maximum absolute yaw rate in the current curve. Once the value
    // has clearly dropped from that peak, emit LEFT_PEAK/RIGHT_PEAK. A rule can
    // then use: LEFT_PEAK -> WAIT 120ms -> BRAKE, which means "after the curve
    // peak, wait 120ms, then act".
    if (a > cornerPeakAbs) {
      cornerPeakAbs = a;
      cornerPeakMs = now;
    }

    uint32_t curveAgeMs = now - cornerStartMs;

    // Curve-hold event: emitted only when the same LEFT/RIGHT curve remains
    // active for CURVE_LONG_MS. This separates a long 180-degree curve from
    // a short 90-degree curve followed by the opposite 90-degree curve.
    // (LEFT_HOLD/RIGHT_HOLD is now the sole long-curve discriminator; the
    //  former *_LONG_PEAK events were removed because PEAK already fires at
    //  curve exit and HOLD already filters by curve length.)
    if (!cornerHoldEmitted && curveAgeMs >= effectiveCurveLongMs()) {
      cornerHoldEmitted = true;
      uint8_t holdSeg = (cornerSign > 0) ? SEG_LEFT_HOLD : SEG_RIGHT_HOLD;
      onSegmentDetected(holdSeg, gz, now);
    }

    bool peakPassed = cornerPeakAbs >= CORNER_TH
                   && (uint32_t)(now - cornerPeakMs) >= effectivePeakAfterMs()
                   && (cornerPeakAbs - a) >= CURVE_PEAK_DROP_DPS;

    if (!cornerPeakEmitted && peakPassed) {
      cornerPeakEmitted = true;
      uint8_t peakSeg = (cornerSign > 0) ? SEG_LEFT_PEAK : SEG_RIGHT_PEAK;
      onSegmentDetected(peakSeg, gz, now);
    }

    // Corner exit debounce only prevents duplicate curve-start events.
    if (a < STRAIGHT_TH) {
      if (straightCandMs == 0) straightCandMs = now;
      if ((now - straightCandMs) >= EXIT_DEBOUNCE_MS) {
        inCorner = false;
        cornerSign = 0;
        straightCandMs = 0;
        straightHoldStartMs = now;
        straightHoldEmitted = false;
      }
    } else {
      straightCandMs = 0;
    }
    return;
  }

  // Straight hold: emit once after a straight has continued below STRAIGHT_TH
  // for STRAIGHT_HOLD_MS. This is deliberately a one-shot per straight section.
  if (a < STRAIGHT_TH) {
    if (straightHoldStartMs == 0) straightHoldStartMs = now;
    if (!straightHoldEmitted && (uint32_t)(now - straightHoldStartMs) >= effectiveStraightHoldMs()) {
      straightHoldEmitted = true;
      onSegmentDetected(SEG_STRAIGHT_HOLD, gz, now);
    }
  } else {
    straightHoldStartMs = 0;
    straightHoldEmitted = false;
  }
}

// ============================================================
// BLE callbacks
// ============================================================
static void onConnected(BLEDevice central) {
  (void)central;
  bleConnected = true;
  statusChar.writeValue(statusValue);
  currentSegChar.writeValue(lastSegment);
}

static void onDisconnected(BLEDevice central) {
  (void)central;
  bleConnected = false;
  // Do not stop the car on BLE disconnect. Production control is local.
}

static void onParamsWritten(BLEDevice central, BLECharacteristic characteristic) {
  (void)central; (void)characteristic;
  if (isRunning) return;  // never mutate control parameters during run

  uint8_t p[30] = {0};
  int len = paramsChar.valueLength();
  if (len < 20) return;
  int nread = len > 30 ? 30 : len;
  paramsChar.readValue(p, nread);

  CORNER_TH        = constrain((float)readU16LE(&p[0]),   50.0f, 800.0f);
  STRAIGHT_TH      = constrain((float)readU16LE(&p[2]),   10.0f, 600.0f);
  WAVE_POS_TH      = constrain((float)readU16LE(&p[4]),   50.0f, 1000.0f);
  WAVE_NEG_TH      = constrain((float)readU16LE(&p[6]),   50.0f, 1000.0f);
  WAVE_WINDOW_MS   = constrain((uint32_t)readU16LE(&p[8]),  40UL, 600UL);
  WAVE_LATCH_MS    = constrain((uint32_t)readU16LE(&p[10]), 100UL, 3000UL);
  WAVE_COOLDOWN_MS = constrain((uint32_t)readU16LE(&p[12]),  50UL, 1500UL);
  EXIT_DEBOUNCE_MS = constrain((uint32_t)readU16LE(&p[14]),   5UL, 100UL);
  NORMAL_DUTY      = p[16];
  RECOVER_DUTY     = p[17];
  START_DUTY       = p[18];
  DEBUG_LIVE_SEG   = (p[19] & 0x01) != 0;
  if (len >= 28) {
    CURVE_PEAK_AFTER_MS = constrain((uint32_t)readU16LE(&p[20]),   5UL, 300UL);
    CURVE_PEAK_DROP_DPS = constrain((float)readU16LE(&p[22]),      5.0f, 400.0f);
    STRAIGHT_HOLD_MS    = constrain((uint32_t)readU16LE(&p[24]),  20UL, 2000UL);
    SEGMENT_COOLDOWN_MS = constrain((uint32_t)readU16LE(&p[26]),   0UL, 500UL);
  }
  if (len >= 30) {
    CURVE_LONG_MS       = constrain((uint32_t)readU16LE(&p[28]),  30UL, 1200UL);
  }

  writeParamsToChar();
}

static void onSpeedWritten(BLEDevice central, BLECharacteristic characteristic) {
  (void)central; (void)characteristic;
  if (isRunning) return;
  NORMAL_DUTY = speedChar.value();
  writeParamsToChar();
}

static void onRulesWritten(BLEDevice central, BLECharacteristic characteristic) {
  (void)central; (void)characteristic;
  if (isRunning) return;  // never mutate rules during run

  int len = rulesChar.valueLength();
  if (len < 2) return;
  uint8_t buf[300] = {0};
  if (len > (int)sizeof(buf)) len = sizeof(buf);
  rulesChar.readValue(buf, len);

  uint8_t count = buf[1];
  if (count > MAX_RULES) count = MAX_RULES;
  // size validation deferred; handled with proto-aware recBytes below.

  rulesCount = count;
  for (uint8_t r = 0; r < MAX_RULES; r++) {
    memset(&rules[r], 0, sizeof(Rule));
    rules[r].peakAfterOverrideMs    = 0xFFFF;
    rules[r].straightHoldOverrideMs = 0xFFFF;
    rules[r].curveLongOverrideMs    = 0xFFFF;
  }

  // protocol version: buf[0] == 2 -> 36B legacy; buf[0] == 3 -> 42B v37.
  uint8_t proto = buf[0];
  uint8_t recBytes = (proto >= 3) ? RULE_BYTES : RULE_BYTES_V2;
  if (len < 2 + count * recBytes) {
    count = (uint8_t)((len - 2) / recBytes);
    rulesCount = count;
  }

  for (uint8_t r = 0; r < count; r++) {
    const uint8_t *q = &buf[2 + r * recBytes];
    Rule &rule = rules[r];
    rule.patternLen = q[0];
    if (rule.patternLen > MAX_PATTERN_LEN) rule.patternLen = MAX_PATTERN_LEN;
    for (uint8_t i = 0; i < MAX_PATTERN_LEN; i++) rule.pattern[i] = q[1 + i];

    rule.actionCount = q[9];
    if (rule.actionCount > MAX_ACTIONS) rule.actionCount = MAX_ACTIONS;
    for (uint8_t i = 0; i < MAX_ACTIONS; i++) {
      rule.actions[i].type = q[10 + i];
      rule.actions[i].durationMs = readU16LE(&q[15 + 2 * i]);
      rule.actions[i].strength = q[25 + i];
    }
    rule.loopMode = (q[30] != 0);
    rule.triggerMode = q[31];
    if (rule.triggerMode > TRIG_ONLY_N) rule.triggerMode = TRIG_EVERY;
    rule.triggerN = q[32] == 0 ? 1 : q[32];
    rule.priority = q[33] == 0 ? (uint8_t)(100 - r) : q[33];
    if (proto >= 3 && recBytes >= 42) {
      rule.peakAfterOverrideMs    = readU16LE(&q[34]);
      rule.straightHoldOverrideMs = readU16LE(&q[36]);
      rule.curveLongOverrideMs    = readU16LE(&q[38]);
    }
    rule.matchIdx = 0;
    rule.matchCount = 0;
    rule.enabled = true;
  }

  resetRuleProgress();
  rulesChar.writeValue(buf, len);
}

static void armRun(uint32_t now) {
  motorBrake();
  isRunning = false;
  resetRuleProgress();
  resetDetection(now);
  setStatus(STATUS_READY);
}

static void startRun(uint32_t now) {
  motorBrake();

  // Recalibrate at start while motor is still off.
  gzBias = calibrateGzBias();
  for (uint8_t i = 0; i < GZ_FILT_N; i++) gzBuf[i] = 0.0f;
  gzIdx = 0;

  logReset();
  resetRuleProgress();
  resetDetection(millis());
  runStartMs = millis();
  lastControlMs = runStartMs;
  activeActionCount = 0;
  activeActionIdx = 0;
  actionActive = false;
  activeRuleIdx = 0xFF;
  activePriority = 0;
  activeTriggerGz = 0.0f;

  isRunning = true;
  setStatus(STATUS_RUNNING);
  motorDuty(START_DUTY);
}

static void stopRun(uint8_t stopStatus) {
  isRunning = false;
  actionActive = false;
  activeActionCount = 0;
  activeActionIdx = 0;
  activeTriggerGz = 0.0f;
  motorBrake();
  setStatus(stopStatus);
  dumpStart();
}

static void onCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
  (void)central; (void)characteristic;
  uint8_t cmd = 0;
  commandChar.readValue(&cmd, 1);
  uint32_t now = millis();

  if (cmd == CMD_STOP) {
    stopRun(STATUS_STOPPED);
  } else if (cmd == CMD_ARM) {
    armRun(now);
  } else if (cmd == CMD_START) {
    startRun(now);
  } else if (cmd == CMD_LOG_REQUEST) {
    if (!isRunning) dumpStart();
  } else if (cmd == CMD_CLEAR_LOG) {
    if (!isRunning) logReset();
  }
}

// ============================================================
// Setup / loop
// ============================================================
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  analogWrite(PWM_PIN, 0);

  pinMode(LED_BUILTIN, OUTPUT);
  ledSet(false);

  pinMode(IMU_EN_PIN, OUTPUT);
  digitalWrite(IMU_EN_PIN, HIGH);
  delay(30);

  Wire.begin();
  Serial.begin(115200);

  if (myIMU.begin() == 0) {
    delay(80);
    gzBias = calibrateGzBias();
  } else {
    gzBias = 0.0f;
  }

  if (BLE.begin()) {
    bleReady = true;
    BLE.setLocalName(DEVICE_NAME);
    BLE.setDeviceName(DEVICE_NAME);

    configService.addCharacteristic(commandChar);
    configService.addCharacteristic(statusChar);
    configService.addCharacteristic(currentSegChar);
    configService.addCharacteristic(speedChar);
    configService.addCharacteristic(paramsChar);
    configService.addCharacteristic(rulesChar);
    configService.addCharacteristic(bulkChar);
    BLE.addService(configService);
    BLE.setAdvertisedService(configService);

    commandChar.setEventHandler(BLEWritten, onCommandWritten);
    speedChar.setEventHandler(BLEWritten, onSpeedWritten);
    paramsChar.setEventHandler(BLEWritten, onParamsWritten);
    rulesChar.setEventHandler(BLEWritten, onRulesWritten);
    BLE.setEventHandler(BLEConnected, onConnected);
    BLE.setEventHandler(BLEDisconnected, onDisconnected);

    commandChar.writeValue((uint8_t)0);
    statusChar.writeValue(statusValue);
    currentSegChar.writeValue((uint8_t)SEG_NONE);
    speedChar.writeValue(NORMAL_DUTY);
    writeParamsToChar();
    uint8_t emptyRules[2] = {1, 0};
    rulesChar.writeValue(emptyRules, 2);
    uint8_t emptyBulk[3] = {0xFF, 0xFF, 0};
    bulkChar.writeValue(emptyBulk, 3);

    BLE.advertise();
  }

  uint32_t now = millis();
  lastControlMs = now;
  lastBlePollMs = now;
  setStatus(STATUS_IDLE);
}

void loop() {
  uint32_t now = millis();

  // 1. Local realtime control first.
  if ((uint32_t)(now - lastControlMs) >= CONTROL_DT_MS) {
    lastControlMs += CONTROL_DT_MS;

    if (isRunning) {
      float gzRaw = myIMU.readFloatGyroZ();
      if (!isnan(gzRaw)) {
        float gz = updateGzFilter(gzRaw - gzBias);

        updateActionEngine(now);
        if (!actionActive && isRunning) {
          // Short soft start: hold START_DUTY briefly, then normal duty.
          uint32_t elapsed = now - runStartMs;
          if (elapsed < SOFT_START_MS) {
            motorDuty(START_DUTY);
          } else {
            motorDuty(NORMAL_DUTY);
          }
        }

        updateSegmentDetection(gz, now);
        updatePendingFires(now);
      }
    } else {
      motorBrake();
    }
  }

  // 2. BLE is lower priority and never required for control.
  now = millis();
  if (bleReady && (uint32_t)(now - lastBlePollMs) >= BLE_POLL_MS) {
    lastBlePollMs = now;
    BLE.poll();

    if (!isRunning) {
      dumpStep();
    }

    if (isRunning && DEBUG_LIVE_SEG && pendingLiveSeg
        && (uint32_t)(now - lastLiveEventMs) >= LIVE_EVENT_MS) {
      lastLiveEventMs = now;
      pendingLiveSeg = false;
      currentSegChar.writeValue(pendingLiveSegValue);
    }
  }

  // Small LED heartbeat. Avoid Serial logging in production.
  if (isRunning) {
    ledSet((millis() / 100) % 2 == 0);
  } else if (bleConnected) {
    ledSet((millis() / 500) % 2 == 0);
  } else {
    ledSet(false);
  }
}
