// ============================================================
// AI制御ミニ四駆 パターン認識ファームウェア
// XIAO MG24 Sense + LSM6DS3 + BLE
//
// セグメント: 直線 / 右90° / 左90° / 右180° / 左180° / ウェーブ
//
// マシン名: MiniYonAI_01 (DEVICE_NAME を変えれば別個体になる)
//
// 【更新内容】
//  - SOFT_START_MS を 800ms → 200ms に短縮
//  - 走行開始(cmd=1) 時に検出状態を完全にリセットし、
//    Web側にも matchIdx=0 / currentSeg=0 を即座に通知
//  - ソフトスタート完了直後に検出状態を「now基準」で再リセット
//    (ソフトスタート中の振動が次のセグメント検出に持ち越されない)
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <math.h>

// ============================================================
// マシン名
// ============================================================
#define DEVICE_NAME "MiniYonAI_01"

// ============================================================
// ハードウェア設定
// ============================================================
static const uint8_t PWM_PIN = PC0;
static const uint8_t IMU_EN_PIN = PD5;

#ifndef LED_BUILTIN
#define LED_BUILTIN LED_RED
#endif

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// ============================================================
// セグメント種別 (簡素化版: 6種類)
// ============================================================
static const uint8_t SEG_NONE      = 0;
static const uint8_t SEG_STRAIGHT  = 1;  // 直線
static const uint8_t SEG_RIGHT_90  = 2;  // 右90°
static const uint8_t SEG_RIGHT_180 = 3;  // 右180°
static const uint8_t SEG_LEFT_90   = 4;  // 左90°
static const uint8_t SEG_LEFT_180  = 5;  // 左180°
static const uint8_t SEG_WAVE      = 6;  // ウェーブ

// ============================================================
// 動作種別
// ============================================================
static const uint8_t ACTION_NONE         = 0;  // 動作なし(認識のみ)
static const uint8_t ACTION_BRAKE        = 1;  // 一時ブレーキ
static const uint8_t ACTION_SPEED_CHANGE = 2;  // 速度変更 (強さで±)
static const uint8_t ACTION_STOP         = 4;  // 即停止
// 旧 ACTION_SLOW(2)/ACTION_BOOST(3) は SPEED_CHANGE(2) に統合 (互換性のため 3 も SPEED_CHANGE 扱い)

// 動作キュー (最大3つ連結)
static const uint8_t MAX_ACTION_COUNT = 3;

// ============================================================
// セグメント検出パラメータ
// ============================================================
// 90°カーブと180°カーブを「ピーク値」と「継続時間」で区別

// ピーク値の閾値 (dps)
static const float CURVE_TH_LOW  = 100.0f;  // ここを超えるとカーブ判定
static const float CURVE_TH_180  = 600.0f;  // ここを超えると180°判定

// 直進判定
static const float STRAIGHT_TH = 50.0f;

// 各セグメントの最小継続時間 (ms)
// 180°は90°より長く回るので継続時間で区別補強
static const uint32_t SEG_90_MIN_MS  = 80;
static const uint32_t SEG_180_MIN_MS = 250;
static const uint32_t STRAIGHT_MIN_MS = 200;

// ウェーブ判定
// マシンがウェーブセクション(立体起伏)を通過する際、左右の傾きが
// 連続的に発生して Z軸ジャイロにも振動が乗る。
// この左右の振動 (3回程度のゼロクロス) を一定時間内に検出する。
// ただし、単なるカーブ(片側に長く曲がる)ではゼロクロスが発生しないので
// 自然に区別できる。
static const uint8_t WAVE_ZEROCROSS_MIN = 3;       // 3回ゼロクロス (右→左→右 or 左→右→左)
static const uint32_t WAVE_WINDOW_MS = 1500;       // 検出ウィンドウ (1.5秒)
static const float WAVE_AMPLITUDE_TH = 200.0f;     // 振幅閾値 (緩めに)
static const float WAVE_AMPLITUDE_MAX = 700.0f;    // これ以上はカーブ扱い

// ============================================================
// パターンマッチング
// ============================================================
static const uint8_t MAX_PATTERN_LEN = 8;
static uint8_t userPattern[MAX_PATTERN_LEN];
static uint8_t userPatternLen = 0;

// 動作キュー: 最大3つの動作を連結可能
static uint8_t userActions[MAX_ACTION_COUNT];
static uint16_t userActionDurations[MAX_ACTION_COUNT];
static uint8_t userActionStrengths[MAX_ACTION_COUNT]; // 1-4 (25%/50%/75%/100%)
static uint8_t userActionCount = 1;

// 現在実行中の動作インデックス
static uint8_t actionQueueIdx = 0;

static uint8_t patternMatchIdx = 0;

// パターン検出済みフラグ (UIの「正解」表示用)
static bool patternMatchedFlag = false;
static uint32_t patternMatchedTimeMs = 0;

// ============================================================
// 走行制御
// ============================================================
static const float VBAT = 4.2f;
static const float NORMAL_V = 4.2f;
// ★変更: 800ms → 200ms に短縮 (GO! ですぐ加速完了するように)
static const uint32_t SOFT_START_MS = 200;
static uint8_t DUTY_NORMAL = 192;  // デフォルト 75% (3/4) ≒ 高速
static uint8_t actionLowSpeedDuty = 100;

static uint8_t calcDuty(float targetV) {
  float r = targetV / VBAT;
  if (r < 0) return 0;
  if (r > 1) return 255;
  return (uint8_t)(r * 255.0f + 0.5f);
}

static inline void motorDuty(uint8_t d) { analogWrite(PWM_PIN, d); }
static inline void motorBrake() { motorDuty(0); }

static const uint8_t MODE_NORMAL = 0;
static const uint8_t MODE_ACTION_ACTIVE = 1;
static uint8_t runMode = MODE_NORMAL;
static uint32_t actionEndMs = 0;
static uint8_t currentAction = ACTION_NONE;

// ============================================================
// IMU・フィルタ
// ============================================================
static const uint8_t GZ_FILT_N = 5;
static const uint16_t GZ_BIAS_SAMPLES = 200;
static const uint32_t LOOP_HZ = 200;
static const uint32_t LOOP_DT_MS = 1000 / LOOP_HZ;

static float gzBias = 0.0f;
static float gzBuf[GZ_FILT_N];
static uint8_t gzIdx = 0;
static float gzSum = 0.0f;

static float updateGzFilter(float gz) {
  gzSum -= gzBuf[gzIdx];
  gzBuf[gzIdx] = gz;
  gzSum += gz;
  gzIdx = (gzIdx + 1) % GZ_FILT_N;
  return gzSum / GZ_FILT_N;
}

// ============================================================
// セグメント検出状態
// ============================================================
static uint8_t currentSegment = SEG_NONE;
static uint8_t segmentCandidate = SEG_NONE;
static uint32_t segmentCandidateStartMs = 0;
static float segmentMaxAbsGz = 0.0f;  // この候補中の最大絶対値

// ウェーブ検出
static uint32_t zeroCrossTimes[16];
static uint8_t zeroCrossIdx = 0;
static int8_t lastGzSign = 0;
static float maxGzAmplitudeRecent = 0.0f;
static uint32_t maxGzTimeRecent = 0;

// ============================================================
// 動作・走行状態
// ============================================================
static bool isRunning = false;
static uint32_t bootStartMs = 0;
static bool softStartDone = false;
static uint32_t lastLoopMs = 0;
static uint32_t lastNotifyMs = 0;
static bool bleConnected = false;

// ============================================================
// LED
// ============================================================
static const uint8_t LED_OFF       = 0;
static const uint8_t LED_IDLE      = 1;
static const uint8_t LED_RUNNING   = 2;
static const uint8_t LED_MATCHING  = 3;
static const uint8_t LED_ACTION    = 4;
static const uint8_t LED_BLE_CONN  = 5;
static const uint8_t LED_MATCHED   = 6;  // 正解時の派手な点滅
static uint8_t ledMode = LED_IDLE;

static inline void ledOn()  { digitalWrite(LED_BUILTIN, HIGH); }
static inline void ledOff() { digitalWrite(LED_BUILTIN, LOW); }

static void updateLed(uint32_t now) {
  bool on = false;
  switch (ledMode) {
    case LED_OFF:      on = false; break;
    case LED_IDLE:     on = ((now / 800) % 2) == 0; break;
    case LED_RUNNING:  on = ((now / 200) % 2) == 0; break;
    case LED_MATCHING: on = ((now / 80)  % 2) == 0; break;
    case LED_ACTION:   on = true; break;
    case LED_BLE_CONN: on = ((now / 100) % 2) == 0; break;
    case LED_MATCHED:  on = ((now / 30)  % 2) == 0; break;
  }
  if (on) ledOn(); else ledOff();
}

// ============================================================
// 検出状態を完全にリセット
// (走行開始時、およびソフトスタート完了直後に呼ぶ)
// ============================================================
static void resetDetectionState(uint32_t now) {
  currentSegment = SEG_NONE;
  segmentCandidate = SEG_NONE;
  segmentCandidateStartMs = now;
  segmentMaxAbsGz = 0.0f;
  zeroCrossIdx = 0;
  lastGzSign = 0;
  maxGzAmplitudeRecent = 0.0f;
  maxGzTimeRecent = now;
}

// ============================================================
// セグメント検出
// ============================================================

// 単発ピーク値ベースで「方向」だけ判定
// 90°か180°かは「継続時間」も合わせて確定する
static uint8_t classifyDirection(float gz, float absGz) {
  if (absGz < STRAIGHT_TH) return SEG_STRAIGHT;
  if (absGz < CURVE_TH_LOW) return SEG_NONE;  // 中間値はノイズ扱い
  if (gz < 0) return SEG_RIGHT_90;  // 右回転(暫定で90°、後で180°と区別)
  return SEG_LEFT_90;               // 左回転(暫定)
}

static void updateWaveDetection(float gz, uint32_t now) {
  int8_t sign = (gz > 30) ? 1 : (gz < -30) ? -1 : 0;
  if (sign != 0 && lastGzSign != 0 && sign != lastGzSign) {
    if (zeroCrossIdx < 16) {
      zeroCrossTimes[zeroCrossIdx++] = now;
    } else {
      for (uint8_t i = 0; i < 15; i++) zeroCrossTimes[i] = zeroCrossTimes[i+1];
      zeroCrossTimes[15] = now;
    }
  }
  if (sign != 0) lastGzSign = sign;

  uint8_t newIdx = 0;
  for (uint8_t i = 0; i < zeroCrossIdx; i++) {
    if ((now - zeroCrossTimes[i]) < WAVE_WINDOW_MS) {
      zeroCrossTimes[newIdx++] = zeroCrossTimes[i];
    }
  }
  zeroCrossIdx = newIdx;

  float absGz = fabsf(gz);
  if (absGz > maxGzAmplitudeRecent) {
    maxGzAmplitudeRecent = absGz;
    maxGzTimeRecent = now;
  }
  if ((now - maxGzTimeRecent) > WAVE_WINDOW_MS) {
    maxGzAmplitudeRecent = absGz;
    maxGzTimeRecent = now;
  }
}

static bool isWaveDetected(uint32_t now) {
  // ゼロクロスが多く、振幅が中程度（カーブほど大きくない）の場合のみウェーブ
  return (zeroCrossIdx >= WAVE_ZEROCROSS_MIN &&
          maxGzAmplitudeRecent > WAVE_AMPLITUDE_TH &&
          maxGzAmplitudeRecent < WAVE_AMPLITUDE_MAX &&
          (now - zeroCrossTimes[0]) < WAVE_WINDOW_MS);
}

// セグメントの遷移を判定
// 候補が一定時間続いたら確定する
// 90° vs 180° は ピーク値 + 継続時間で判定
static bool updateSegmentTransition(float gz, uint32_t now) {
  float absGz = fabsf(gz);

  // ウェーブ優先判定
  if (isWaveDetected(now)) {
    if (currentSegment != SEG_WAVE) {
      currentSegment = SEG_WAVE;
      segmentCandidate = SEG_WAVE;
      segmentMaxAbsGz = absGz;
      return true;
    }
    return false;
  }

  uint8_t directionCandidate = classifyDirection(gz, absGz);

  if (directionCandidate == SEG_NONE) {
    // 中間値、現状維持
    return false;
  }

  // 候補が変わった
  if (directionCandidate != segmentCandidate) {
    segmentCandidate = directionCandidate;
    segmentCandidateStartMs = now;
    segmentMaxAbsGz = absGz;
    return false;
  }

  // 同じ候補が継続中、最大値を更新
  if (absGz > segmentMaxAbsGz) {
    segmentMaxAbsGz = absGz;
  }

  // 候補がカーブの場合、180°か90°かを判定
  uint8_t finalSegment = directionCandidate;
  bool is180 = (segmentMaxAbsGz > CURVE_TH_180);
  uint32_t durationMs = now - segmentCandidateStartMs;

  if (directionCandidate == SEG_RIGHT_90) {
    if (is180 && durationMs >= SEG_180_MIN_MS) {
      finalSegment = SEG_RIGHT_180;
    } else if (durationMs >= SEG_90_MIN_MS) {
      finalSegment = SEG_RIGHT_90;
    } else {
      return false;  // まだ確定しない
    }
  } else if (directionCandidate == SEG_LEFT_90) {
    if (is180 && durationMs >= SEG_180_MIN_MS) {
      finalSegment = SEG_LEFT_180;
    } else if (durationMs >= SEG_90_MIN_MS) {
      finalSegment = SEG_LEFT_90;
    } else {
      return false;
    }
  } else if (directionCandidate == SEG_STRAIGHT) {
    if (durationMs < STRAIGHT_MIN_MS) return false;
  }

  // 確定
  if (finalSegment != currentSegment) {
    currentSegment = finalSegment;
    return true;
  }
  return false;
}

// ============================================================
// パターンマッチング
// ============================================================

// セグメントのマッチ判定
// 90°と180°は別物として厳密に判定
static bool segmentMatches(uint8_t expected, uint8_t actual) {
  return expected == actual;
}

static void startNextActionInQueue(uint32_t now) {
  if (actionQueueIdx >= userActionCount) {
    // 全動作完了 → 通常走行に戻る
    runMode = MODE_NORMAL;
    currentAction = ACTION_NONE;
    motorDuty(DUTY_NORMAL);
    Serial.println("[ACT] all done → resume");
    return;
  }
  currentAction = userActions[actionQueueIdx];
  uint16_t dur = userActionDurations[actionQueueIdx];
  runMode = MODE_ACTION_ACTIVE;
  actionEndMs = now + dur;
  Serial.print("[ACT] start #");
  Serial.print(actionQueueIdx + 1);
  Serial.print(" type=");
  Serial.print(currentAction);
  Serial.print(" dur=");
  Serial.print(dur);
  Serial.println("ms");
}

static void executeAction(uint32_t now) {
  patternMatchedFlag = true;
  patternMatchedTimeMs = now;

  // 動作キューの最初の動作を開始
  if (userActionCount > 0 && userActions[0] != ACTION_NONE) {
    actionQueueIdx = 0;
    startNextActionInQueue(now);
  }
}

static void updatePatternMatch(uint32_t now) {
  if (userPatternLen == 0) return;
  if (runMode != MODE_NORMAL) return;
  if (patternMatchedFlag) return;  // 既に正解済みなら判定しない

  uint8_t expected = userPattern[patternMatchIdx];

  if (segmentMatches(expected, currentSegment)) {
    // 期待通り → 進捗を進める
    patternMatchIdx++;
    if (patternMatchIdx >= userPatternLen) {
      // 全マッチ完了
      executeAction(now);
      patternMatchIdx = 0;
      zeroCrossIdx = 0;
      maxGzAmplitudeRecent = 0;
    }
  } else {
    // 期待と違う場合
    // 直進は許容、index 維持 (パターン中の経由扱い)
    if (currentSegment == SEG_STRAIGHT && patternMatchIdx > 0) {
      // index 維持
      return;
    }
    // それ以外は進捗を 0 にリセットしつつ、現在のセグメントが1手目に該当するか即判定
    // (これにより、パターン途中で違うセグメントが来ても、そのセグメントが
    //  パターンの先頭に該当するなら新たなマッチング開始ができる)
    patternMatchIdx = 0;
    if (segmentMatches(userPattern[0], currentSegment)) {
      patternMatchIdx = 1;
      if (patternMatchIdx >= userPatternLen) {
        executeAction(now);
        patternMatchIdx = 0;
        zeroCrossIdx = 0;
        maxGzAmplitudeRecent = 0;
      }
    }
  }
}

// ============================================================
// IMU
// ============================================================
static float calibrateGzBias() {
  float sum = 0;
  for (uint16_t i = 0; i < GZ_BIAS_SAMPLES; i++) {
    sum += myIMU.readFloatGyroZ();
    delay(2);
  }
  return sum / GZ_BIAS_SAMPLES;
}

// ============================================================
// BLE
// ============================================================
BLEService configService("12345678-1234-5678-1234-56789abcdef0");

BLECharacteristic patternChar(
  "12345678-1234-5678-1234-56789abcdef1",
  BLERead | BLEWrite,
  16
);
// 動作配列: [count, action1, action2, action3, dur1_lo, dur1_hi, dur2_lo, dur2_hi, dur3_lo, dur3_hi, str1, str2, str3]
// 計13バイト
BLECharacteristic actionChar(
  "12345678-1234-5678-1234-56789abcdef2",
  BLERead | BLEWrite,
  13
);
// 旧 actionDurationChar は使わないが互換性のため残しておく
BLEUnsignedShortCharacteristic actionDurationChar(
  "12345678-1234-5678-1234-56789abcdef3",
  BLERead | BLEWrite
);
BLEByteCharacteristic commandChar(
  "12345678-1234-5678-1234-56789abcdef4",
  BLERead | BLEWrite
);
BLEByteCharacteristic statusChar(
  "12345678-1234-5678-1234-56789abcdef5",
  BLERead | BLENotify
);
BLEByteCharacteristic currentSegChar(
  "12345678-1234-5678-1234-56789abcdef6",
  BLERead | BLENotify
);
BLEByteCharacteristic matchIdxChar(
  "12345678-1234-5678-1234-56789abcdef7",
  BLERead | BLENotify
);
// 「パターン全マッチした」フラグ通知
BLEByteCharacteristic matchedFlagChar(
  "12345678-1234-5678-1234-56789abcdef8",
  BLERead | BLENotify
);
// 走行速度設定 (0~255 の duty 値)
BLEByteCharacteristic speedChar(
  "12345678-1234-5678-1234-56789abcdefa",
  BLERead | BLEWrite
);

// ============================================================
// BLE ハンドラ
// ============================================================
static void onPatternWritten(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t buf[16];
  int len = patternChar.valueLength();
  if (len > 16) len = 16;
  patternChar.readValue(buf, len);

  userPatternLen = (buf[0] > MAX_PATTERN_LEN) ? MAX_PATTERN_LEN : buf[0];
  for (uint8_t i = 0; i < userPatternLen; i++) {
    userPattern[i] = buf[i + 1];
  }
  patternMatchIdx = 0;
  patternMatchedFlag = false;

  // デバッグ出力
  Serial.print("[PATTERN] len=");
  Serial.print(userPatternLen);
  Serial.print(" data=[");
  for (uint8_t i = 0; i < userPatternLen; i++) {
    Serial.print(userPattern[i]);
    if (i < userPatternLen - 1) Serial.print(",");
  }
  Serial.println("]");
}

static void onActionWritten(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t buf[13];
  int len = actionChar.valueLength();
  if (len > 13) len = 13;
  actionChar.readValue(buf, len);

  // [count, a1, a2, a3, d1lo, d1hi, d2lo, d2hi, d3lo, d3hi, s1, s2, s3]
  uint8_t count = buf[0];
  if (count > MAX_ACTION_COUNT) count = MAX_ACTION_COUNT;
  if (count < 1) count = 1;
  userActionCount = count;

  for (uint8_t i = 0; i < MAX_ACTION_COUNT; i++) {
    if (i < count && (1 + i) < len) {
      userActions[i] = buf[1 + i];
    } else {
      userActions[i] = ACTION_NONE;
    }
    uint16_t durLow = (4 + i*2 < len) ? buf[4 + i*2] : 0;
    uint16_t durHigh = (5 + i*2 < len) ? buf[5 + i*2] : 0;
    userActionDurations[i] = durLow | (durHigh << 8);
    // strength (デフォルト2 = 50%)
    if (10 + i < len) {
      userActionStrengths[i] = buf[10 + i];
      if (userActionStrengths[i] < 1) userActionStrengths[i] = 1;
      if (userActionStrengths[i] > 4) userActionStrengths[i] = 4;
    } else {
      userActionStrengths[i] = 2; // デフォルト50%
    }
  }

  // デバッグ出力
  Serial.print("[ACTION] count=");
  Serial.print(userActionCount);
  for (uint8_t i = 0; i < userActionCount; i++) {
    Serial.print(" #");
    Serial.print(i+1);
    Serial.print("=");
    Serial.print(userActions[i]);
    Serial.print("/");
    Serial.print(userActionDurations[i]);
    Serial.print("ms/str");
    Serial.print(userActionStrengths[i]);
  }
  Serial.println();
}

// 旧 actionDurationChar (互換性のため残す、無視する)
static void onActionDurationWritten(BLEDevice central, BLECharacteristic characteristic) {
  // 何もしない (新しい actionChar で全て受け取る)
}

static void onSpeedWritten(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t v = speedChar.value();
  DUTY_NORMAL = v;
  Serial.print("[SPEED] DUTY_NORMAL=");
  Serial.println(DUTY_NORMAL);
}

static void onCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t cmd = commandChar.value();
  uint32_t now = millis();
  if (cmd == 0) {
    isRunning = false;
    motorBrake();
  } else if (cmd == 1) {
    // ★走行開始: モーターを必ず 0 から開始 + 検出状態を完全にリセット
    motorDuty(0);
    isRunning = true;
    bootStartMs = now;
    softStartDone = false;
    runMode = MODE_NORMAL;
    patternMatchIdx = 0;
    resetDetectionState(now);
    patternMatchedFlag = false;

    // ★Web側にも即座に状態リセットを通知
    matchedFlagChar.writeValue(0);
    matchIdxChar.writeValue(0);
    currentSegChar.writeValue(0);

    Serial.println("[CMD] start running");
  } else if (cmd == 2) {
    isRunning = false;
    motorBrake();
    patternMatchIdx = 0;
    patternMatchedFlag = false;
    matchedFlagChar.writeValue(0);
    matchIdxChar.writeValue(0);
    currentSegChar.writeValue(0);
    Serial.println("[CMD] reset");
  }
}

static void onBleConnected(BLEDevice central) {
  bleConnected = true;
}

static void onBleDisconnected(BLEDevice central) {
  bleConnected = false;
}

// ============================================================
// setup
// ============================================================
void setup() {
  // モーター制御ピンを最優先で OUTPUT + LOW に
  // (電源投入直後に PWM が浮かないようにする)
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  analogWrite(PWM_PIN, 0);

  pinMode(LED_BUILTIN, OUTPUT);
  ledOff();
  pinMode(IMU_EN_PIN, OUTPUT);
  digitalWrite(IMU_EN_PIN, HIGH);
  delay(20);
  Wire.begin();
  Serial.begin(115200);

  if (myIMU.begin() != 0) {
    gzBias = 0;
  } else {
    delay(100);
    gzBias = calibrateGzBias();
  }
  for (uint8_t i = 0; i < GZ_FILT_N; i++) gzBuf[i] = 0;

  if (!BLE.begin()) {
    // BLE 失敗してもLED点滅で続行可能
  }

  BLE.setLocalName(DEVICE_NAME);
  BLE.setDeviceName(DEVICE_NAME);

  configService.addCharacteristic(patternChar);
  configService.addCharacteristic(actionChar);
  configService.addCharacteristic(actionDurationChar);
  configService.addCharacteristic(commandChar);
  configService.addCharacteristic(statusChar);
  configService.addCharacteristic(currentSegChar);
  configService.addCharacteristic(matchIdxChar);
  configService.addCharacteristic(matchedFlagChar);
  configService.addCharacteristic(speedChar);

  BLE.addService(configService);
  BLE.setAdvertisedService(configService);

  uint8_t emptyPattern[16] = {0};
  patternChar.writeValue(emptyPattern, 16);

  // 動作配列の初期値: 1個の即停止(0ms、strength=2)
  uint8_t initialAction[13] = {1, ACTION_STOP, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2};
  actionChar.writeValue(initialAction, 13);
  userActionCount = 1;
  userActions[0] = ACTION_STOP;
  userActionDurations[0] = 0;
  userActionStrengths[0] = 2;
  for (uint8_t i = 1; i < MAX_ACTION_COUNT; i++) {
    userActions[i] = ACTION_NONE;
    userActionDurations[i] = 0;
    userActionStrengths[i] = 2;
  }

  actionDurationChar.writeValue(0);
  commandChar.writeValue(0);
  statusChar.writeValue(0);
  currentSegChar.writeValue(0);
  matchIdxChar.writeValue(0);
  matchedFlagChar.writeValue(0);
  speedChar.writeValue(DUTY_NORMAL);

  patternChar.setEventHandler(BLEWritten, onPatternWritten);
  actionChar.setEventHandler(BLEWritten, onActionWritten);
  actionDurationChar.setEventHandler(BLEWritten, onActionDurationWritten);
  commandChar.setEventHandler(BLEWritten, onCommandWritten);
  speedChar.setEventHandler(BLEWritten, onSpeedWritten);

  BLE.setEventHandler(BLEConnected, onBleConnected);
  BLE.setEventHandler(BLEDisconnected, onBleDisconnected);

  BLE.advertise();

  uint32_t now = millis();
  lastLoopMs = now;
  lastNotifyMs = now;
}

// ============================================================
// loop
// ============================================================
void loop() {
  uint32_t now = millis();
  BLE.poll();

  // 正解後のLEDフラッシュは1秒で終了
  if (patternMatchedFlag && (now - patternMatchedTimeMs) > 1000) {
    // フラッシュ表示は終わるが、Web側にフラグは残しておく
    // (Web側のリセットコマンドでクリアされる)
  }

  updateLed(now);

  if (now - lastLoopMs < LOOP_DT_MS) return;
  lastLoopMs += LOOP_DT_MS;

  if (!isRunning) {
    motorBrake();
    if (patternMatchedFlag && (now - patternMatchedTimeMs) < 1000) {
      ledMode = LED_MATCHED;
    } else {
      ledMode = bleConnected ? LED_BLE_CONN : LED_IDLE;
    }
    return;
  }

  if (!softStartDone) {
    uint32_t elapsed = now - bootStartMs;
    if (elapsed >= SOFT_START_MS) {
      softStartDone = true;
      motorDuty(DUTY_NORMAL);
      // ★ソフトスタート完了直後に検出状態を「now基準」で再リセット
      //   (加速中のジャイロ揺れを次のセグメント検出に持ち越さない)
      resetDetectionState(now);
    } else {
      // DUTY_NORMAL を目標としてゆっくり立ち上げる (0 → DUTY_NORMAL)
      float p = (float)elapsed / (float)SOFT_START_MS;
      uint16_t targetDuty = (uint16_t)((float)DUTY_NORMAL * p);
      if (targetDuty > 255) targetDuty = 255;
      motorDuty((uint8_t)targetDuty);
      ledMode = LED_RUNNING;
      return;
    }
  }

  float gzRaw = myIMU.readFloatGyroZ();
  if (isnan(gzRaw)) return;
  float gz = gzRaw - gzBias;
  float gzFilt = updateGzFilter(gz);

  updateWaveDetection(gzFilt, now);

  bool changed = updateSegmentTransition(gzFilt, now);
  if (changed) {
    currentSegChar.writeValue(currentSegment);
    updatePatternMatch(now);
    matchIdxChar.writeValue(patternMatchIdx);
    if (patternMatchedFlag) {
      matchedFlagChar.writeValue(1);
    }

    // デバッグ: セグメント変化とパターンマッチング状況
    const char* segName = "?";
    switch (currentSegment) {
      case SEG_STRAIGHT:  segName = "STRAIGHT"; break;
      case SEG_RIGHT_90:  segName = "RIGHT_90"; break;
      case SEG_RIGHT_180: segName = "RIGHT_180"; break;
      case SEG_LEFT_90:   segName = "LEFT_90"; break;
      case SEG_LEFT_180:  segName = "LEFT_180"; break;
      case SEG_WAVE:      segName = "WAVE"; break;
    }
    Serial.print("[SEG] ");
    Serial.print(segName);
    Serial.print(" gz=");
    Serial.print(gzFilt, 1);
    Serial.print(" matchIdx=");
    Serial.print(patternMatchIdx);
    Serial.print("/");
    Serial.println(userPatternLen);
  }

  switch (runMode) {
    case MODE_NORMAL:
      motorDuty(DUTY_NORMAL);
      if (patternMatchedFlag && (now - patternMatchedTimeMs) < 1000) {
        ledMode = LED_MATCHED;
      } else {
        ledMode = (patternMatchIdx > 0) ? LED_MATCHING : LED_RUNNING;
      }
      break;

    case MODE_ACTION_ACTIVE:
      // ACTION_STOP の特別処理:
      // - これがキューの最後 → 完全停止 (走行終了)
      // - キューの途中 → 指定時間 motorBrake してから次のアクションへ
      // - 0ms かつ最後 → 即座に走行終了
      if (currentAction == ACTION_STOP) {
        bool isLastAction = (actionQueueIdx >= userActionCount - 1);
        uint16_t dur = userActionDurations[actionQueueIdx];

        if (isLastAction) {
          // 最後のアクション: 即座に走行終了 (時間に関係なく)
          motorBrake();
          isRunning = false;
          runMode = MODE_NORMAL;
          currentAction = ACTION_NONE;
          ledMode = LED_ACTION;
          break;
        }

        // 途中のアクション: 指定時間ブレーキしてから次へ
        if ((int32_t)(now - actionEndMs) >= 0) {
          actionQueueIdx++;
          if (actionQueueIdx < userActionCount && userActions[actionQueueIdx] != ACTION_NONE) {
            startNextActionInQueue(now);
          } else {
            runMode = MODE_NORMAL;
            currentAction = ACTION_NONE;
            motorDuty(DUTY_NORMAL);
          }
        } else {
          motorBrake();  // 指定時間中は停止
          ledMode = LED_ACTION;
        }
        break;
      }

      // 通常の動作 (BRAKE / SPEED_CHANGE)
      if ((int32_t)(now - actionEndMs) >= 0) {
        // 現在の動作終了 → 次の動作へ進む
        actionQueueIdx++;
        if (actionQueueIdx < userActionCount && userActions[actionQueueIdx] != ACTION_NONE) {
          startNextActionInQueue(now);
        } else {
          runMode = MODE_NORMAL;
          currentAction = ACTION_NONE;
          motorDuty(DUTY_NORMAL);
        }
      } else {
        ledMode = LED_ACTION;
        // strength: 1=25%(64), 2=50%(128), 3=75%(192), 4=100%(255)
        uint8_t strength = userActionStrengths[actionQueueIdx];
        if (strength < 1) strength = 1;
        if (strength > 4) strength = 4;
        if (currentAction == ACTION_BRAKE) {
          motorBrake();
        } else if (currentAction == ACTION_SPEED_CHANGE || currentAction == 3) {
          // 速度変更: 強さに応じた目標 DUTY を直接指定
          uint8_t targetDuty;
          if (strength == 1)      targetDuty = 64;   // 25%
          else if (strength == 2) targetDuty = 128;  // 50%
          else if (strength == 3) targetDuty = 192;  // 75%
          else                    targetDuty = 255;  // 100%
          motorDuty(targetDuty);
        } else {
          motorDuty(DUTY_NORMAL);
        }
      }
      break;
  }

  if ((now - lastNotifyMs) > 50) {
    lastNotifyMs = now;
    statusChar.writeValue((isRunning ? 1 : 0) | (runMode << 1));
  }
}
