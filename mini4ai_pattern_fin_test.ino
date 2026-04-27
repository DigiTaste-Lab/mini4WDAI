// ============================================================
// AI制御ミニ四駆 パターン認識ファームウェア
// XIAO MG24 Sense + LSM6DS3 + BLE
//
// セグメント: 直線 / 右90° / 左90° / 右180° / 左180° / ウェーブ
//
// マシン名: MiniYonAI_01 (DEVICE_NAME を変えれば別個体になる)
//
// 【更新内容 v2】
//  - 90°/180°判定を「ピーク値」から「累積回転角(∫gz dt)」ベースに変更
//    ・物理的に正しい判別量。R≈一定の前提で 90° と 180° は通過時間に
//      比例して累積角度が約2倍になる。ピークだけだと識別困難。
//  - カーブ抜け後のウェーブ誤判定を抑制
//    ・カーブ確定後 WAVE_COOLDOWN_MS の間はウェーブ判定をブロック
//    ・確定時にゼロクロス履歴と振幅履歴を同時にクリア
//    ・WAVE_AMPLITUDE_MAX を超えるサンプルを観測したら zeroCross をリセット
//  - BLEで生ジャイロデータと内部状態を 50Hz で配信 (gyroChar 追加)
//    [gz_x10 (i16), accumAngle_deg (i16), maxAmp_x10 (u16),
//     zeroCrossCount (u8), segCandidate (u8)] = 8 bytes
//    ウェブ側でリアルタイム可視化し、判定ロジックの妥当性検証ができる
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
// セグメント検出パラメータ (累積角度ベース v2)
// ============================================================
// 90° と 180° は「累積回転角 ∫gz dt の符号付きの値」で判別する。
// 重要: 候補を「右/左」で分けて持つのではなく、「カーブイベント中か直進か」
//       だけで状態管理し、向きは積分結果の符号で確定する。
//       こうすると 180° の途中で一瞬反対符号のピーク (車体の揺れ等) が出ても、
//       積分が少し戻るだけで累積を失わない (= 90° 誤判定が起きない)。

// カーブ「開始」と判定する角速度しきい値 (dps)
static const float CURVE_TH_LOW  = 100.0f;  // |gz| がこれを超えたらカーブイベント開始

// ============================================================
// セグメント検出のしきい値 (Z軸 gz 単体、シンプルな閾値判定)
// ============================================================
// 検出ロジックの全体像:
//   1. WAVE 判定 (最優先): 過去 WAVE_WINDOW_MS 内のサンプル振幅
//      (max - min) が WAVE_AMPLITUDE_TH を超えたら WAVE
//   2. カーブ判定: |gz| > CURVE_TH_LOW でカーブ開始、|gz| < STRAIGHT_TH が
//      SEG_CURVE_END_MS 続いたらカーブ終了。終了時の累積角度で 90/180 判定
//   3. 直進判定: 静かな状態が STRAIGHT_MIN_MS 続いたら STRAIGHT

// カーブ出口 / 直進判定: |gz| < STRAIGHT_TH が続いている時間で「直進」「カーブ終了」を判定
static const float STRAIGHT_TH = 50.0f;

// 累積角度の判定しきい値 (deg)
static const float ACCUM_ANGLE_90_MIN  = 50.0f;   // これ未満は無効カーブ扱い (ノイズ)
static const float ACCUM_ANGLE_180_MIN = 110.0f;  // これ以上で180°、未満で90°

// カーブイベント終了判定:
// |gz| < STRAIGHT_TH の状態が SEG_CURVE_END_MS 続いたらカーブ終了とみなして確定
static const uint32_t SEG_CURVE_END_MS = 100;
// 直進確定: SEG_CURVE_END_MS よりさらに長く静かなら「直線」セグメント発火
static const uint32_t STRAIGHT_MIN_MS  = 200;

// 走行開始直後の「最初の直線」を確実に検出するためのフォールバック時間 (ms)
// 高速走行時、ソフトスタート完了直後の加速振動で gz が STRAIGHT_TH を散発的に
// 超えてしまい、STRAIGHT_MIN_MS 連続して静かな期間が確保できないことがある。
// その結果スタート直後の直線が見落とされ、最初に検出されるセグメントが
// カーブから始まってしまうのを防ぐため、検出開始から一定時間経過後に
// まだカーブイベントに入っていなければ強制的に STRAIGHT を確定する。
static const uint32_t STRAIGHT_INITIAL_FORCE_MS = 150;

// ----- ウェーブ (波) 判定 -----
// シンプル仕様: 過去 WAVE_WINDOW_MS (200ms) のサンプル履歴の中で、
//   最大値 max(gz) と最小値 min(gz) の差 (peak-to-peak 振幅) が
//   WAVE_AMPLITUDE_TH を超えたら WAVE と判定する。
// これにより:
//   ・低速走行で +200/-300 のように両側に振れる波 → 振幅 500、検出 ✓
//   ・高速走行で +560/-69 のように主方向ディップする波 → 振幅 629、検出 ✓
//   ・純粋カーブ (ピーク +400 → 0 への単調減速) → 振幅 ~400、検出されない ✓
//   ・直線 (gz が ±50 内に収まる) → 振幅 ~100、検出されない ✓
//
//   WAVE_AMPLITUDE_TH : 振幅閾値 (max - min)。実機で調整しやすいように単一の値
//   WAVE_WINDOW_MS    : 振幅を見る時間窓
//   WAVE_COOLDOWN_MS  : 確定後の再判定ブロック (連続発火を防ぐ)
//   WAVE_HISTORY_LEN  : 履歴バッファのサンプル数 (200Hz × 200ms = 40 程度欲しい)
static const float    WAVE_AMPLITUDE_TH = 450.0f;
static const uint32_t WAVE_WINDOW_MS    = 200;
static const uint32_t WAVE_COOLDOWN_MS  = 500;
static const uint8_t  WAVE_HISTORY_LEN  = 50;

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
// セグメント検出状態 (v2: カーブイベント方式)
// ============================================================
static uint8_t currentSegment = SEG_NONE;

// カーブイベント中のフラグ
//   true  = |gz| が CURVE_TH_LOW を超えてからまだ「沈静化」していない
//   false = 直進状態 (gz < STRAIGHT_TH が SEG_CURVE_END_MS 以上続いた状態)
static bool inCurveEvent = false;

// 直近で |gz| > STRAIGHT_TH を観測した時刻
//   この時刻からの経過時間で「カーブ終了」と「直進確定」を判定する
static uint32_t lastHighActivityMs = 0;

// カーブイベント開始から積分した符号付き累積角度 (deg)
//   正 = 左回転、負 = 右回転
static float continuousCurveAccum = 0.0f;

// 直近に確定したカーブの累積角度 (デバッグ表示用にリセット直前の値を退避)
static float lastCommitAccum = 0.0f;

// 直近の積分時刻 (dt 計算用)
static uint32_t lastIntegrateMs = 0;

// 検出開始時刻 (resetDetectionState で更新)
//   走行開始 / ソフトスタート完了のタイミングからの経過時間で、
//   「最初の直線フォールバック」発火タイミングを決める基準となる。
static uint32_t detectionStartMs = 0;

// セグメント表示用 (BLE notify でウェブ側に送る簡易ステート)
//   0=NONE, 1=STRAIGHT, 2=右回転中, 4=左回転中, 6=WAVE 確定中
static uint8_t segmentCandidate = SEG_NONE;

// ----- ウェーブ判定用の状態 -----
// 過去 WAVE_HISTORY_LEN サンプルの (時刻, gz) を保持するリングバッファ。
// WAVE_WINDOW_MS 以内のサンプルから max/min を計算して振幅を見る。
static float    waveHistGz[WAVE_HISTORY_LEN];
static uint32_t waveHistMs[WAVE_HISTORY_LEN];
static uint8_t  waveHistIdx = 0;       // 次に書き込む位置
static uint8_t  waveHistCount = 0;     // 蓄積されたサンプル数 (最大 WAVE_HISTORY_LEN)
static uint32_t waveCooldownUntilMs = 0;  // この時刻まで WAVE 再発火を抑制

// ============================================================
// 動作・走行状態
// ============================================================
static bool isRunning = false;
static uint32_t bootStartMs = 0;

// ============================================================
// 高解像度バルクバッファ (200Hz相当)
// ============================================================
// 走行中、各メインループ tick (200Hz) で gz をリングバッファに追加。
// 走行終了時にホスト(Web側)へ BLE bulkChar 経由でまとめて送信する。
// これにより BLE notify のリアルタイム送信レート (実効 ~12-15Hz) では
// 観測しきれない高速な現象 (波・スパイク等) を、走行後に解析できる。

// 8000サンプル × 200Hz = 40秒分。1サンプル 2bytes (int16) = 16KB RAM。
// XIAO MG24 の RAM (256KB) なら余裕。
static const uint16_t BULK_BUF_LEN = 8000;
static int16_t bulkBuf[BULK_BUF_LEN];   // gz × 10 を int16 で保持
static uint16_t bulkBufWriteIdx = 0;    // 次に書き込む位置
static uint16_t bulkBufCount = 0;       // 蓄積されたサンプル数 (最大 BULK_BUF_LEN)
static bool bulkOverflowed = false;     // バッファあふれフラグ

// ダンプ転送状態
//   bulkSendActive=true の間、メインループで bulkChar.writeValue を分割送信する
static bool bulkSendActive = false;
static uint16_t bulkSendIdx = 0;        // 次に送信する bulkBuf の先頭インデックス
static uint16_t bulkSendSeq = 0;        // パケット seqNum
static uint32_t bulkSendLastMs = 0;     // 直近の送信時刻 (送信間隔調整用)
// 送信パラメータ:
//   ・8000 サンプル / 9 サンプル/パケット ≒ 889 パケット
//   ・889 × 6ms = 約 5.3 秒 で全送信完了 (40秒の走行ログ)
//   ・短い走行ならもっと早く完了する
static const uint32_t BULK_SEND_INTERVAL_MS = 6;   // パケット送信間隔
static const uint8_t  BULK_SAMPLES_PER_PACKET = 9; // 1パケットあたりサンプル数 (3+18=21 bytes)
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
  inCurveEvent = false;
  lastHighActivityMs = now;
  continuousCurveAccum = 0.0f;
  lastIntegrateMs = now;
  detectionStartMs = now;
  segmentCandidate = SEG_NONE;
  waveHistIdx = 0;
  waveHistCount = 0;
  waveCooldownUntilMs = now;
}

// ============================================================
// セグメント検出 (シンプル設計、Z軸 gz の閾値判定のみ)
// ============================================================
//
// 設計思想:
//   ・カーブは |gz| > CURVE_TH_LOW で開始、|gz| < STRAIGHT_TH の静寂が
//     SEG_CURVE_END_MS 続いたら終了。終了時の累積角度で 90/180 を分類。
//   ・WAVE は「短時間 (WAVE_WINDOW_MS) 以内に gz の正ピークと負ピーク両方が
//     見える」だけで検出。カーブ中・直進中どちらでも判定可能。
//   ・WAVE が検出されたら、進行中のカーブイベントは破棄する (波で姿勢が
//     乱れた = カーブとしての累積は信頼できない)。
//

// ----- ウェーブ判定 (振幅ベース) -----
// 戻り値: ウェーブと判定したら true
// 履歴バッファに (now, gz) を追加してから、WAVE_WINDOW_MS 以内のサンプルだけを
// 見て max/min を算出。peak-to-peak 振幅が WAVE_AMPLITUDE_TH を超えたら波。
static bool isWaveDetected(uint32_t now, float gz) {
  // 履歴に追加
  waveHistGz[waveHistIdx] = gz;
  waveHistMs[waveHistIdx] = now;
  waveHistIdx = (waveHistIdx + 1) % WAVE_HISTORY_LEN;
  if (waveHistCount < WAVE_HISTORY_LEN) waveHistCount++;

  // クールダウン中は判定無効化 (連続発火防止)
  if ((int32_t)(now - waveCooldownUntilMs) < 0) return false;

  // WAVE_WINDOW_MS 以内のサンプルから max/min を計算
  float gMax = -1e9f, gMin = 1e9f;
  uint8_t inWindow = 0;
  for (uint8_t i = 0; i < waveHistCount; i++) {
    if ((now - waveHistMs[i]) <= WAVE_WINDOW_MS) {
      if (waveHistGz[i] > gMax) gMax = waveHistGz[i];
      if (waveHistGz[i] < gMin) gMin = waveHistGz[i];
      inWindow++;
    }
  }
  // 窓内のサンプルが少なすぎると判定しない (最低 5 サンプル)
  if (inWindow < 5) return false;

  return ((gMax - gMin) >= WAVE_AMPLITUDE_TH);
}

// 累積角度 → セグメント種別
//   |accum| < ACCUM_ANGLE_90_MIN  → 無効 (ノイズ)
//   ACCUM_ANGLE_90_MIN ≤ |accum| < ACCUM_ANGLE_180_MIN → 90°
//   ACCUM_ANGLE_180_MIN ≤ |accum|                     → 180°
//   accum の符号で 左/右 を区別
static uint8_t classifyAccum(float accum) {
  float absAccum = fabsf(accum);
  if (absAccum < ACCUM_ANGLE_90_MIN) return SEG_NONE;
  bool is180 = (absAccum >= ACCUM_ANGLE_180_MIN);
  if (accum < 0) {
    return is180 ? SEG_RIGHT_180 : SEG_RIGHT_90;
  } else {
    return is180 ? SEG_LEFT_180 : SEG_LEFT_90;
  }
}

// メインの状態遷移ロジック
//
// 戻り値: currentSegment が変わった (= ウェブ側へ通知すべき) なら true
//
// 処理順序:
//   1. ウェーブ判定 (最優先) → カーブイベント破棄して WAVE 確定
//   2. 「最初の直線」フォールバック (走行開始直後の特殊救済)
//   3. カーブイベントの管理 (開始/積分/終了確定)
//   4. 直進確定 (静寂が続いたら STRAIGHT)
static bool updateSegmentTransition(float gz, uint32_t now) {
  float absGz = fabsf(gz);

  // dt 計算
  float dtSec = (float)(now - lastIntegrateMs) / 1000.0f;
  if (dtSec < 0.0f) dtSec = 0.0f;
  if (dtSec > 0.1f) dtSec = 0.1f;
  lastIntegrateMs = now;

  // ----- 1. ウェーブ判定 (最優先) -----
  // 過去 WAVE_WINDOW_MS 以内のサンプルから max/min を算出し、
  // 振幅 (max-min) が WAVE_AMPLITUDE_TH を超えたら波と判定する。
  // 確定したら進行中のカーブイベントは破棄。
  if (isWaveDetected(now, gz)) {
    if (currentSegment != SEG_WAVE) {
      currentSegment = SEG_WAVE;
      segmentCandidate = SEG_WAVE;
      // カーブイベント中だったら破棄 (波で姿勢が乱れたので累積は信頼できない)
      inCurveEvent = false;
      continuousCurveAccum = 0.0f;
      // クールダウンを設定して連続発火を防ぐ
      waveCooldownUntilMs = now + WAVE_COOLDOWN_MS;
      // 履歴もクリア (次の WAVE 判定は新たに gz サンプルが溜まってから)
      waveHistIdx = 0;
      waveHistCount = 0;
      Serial.print("[WAVE] gz=");
      Serial.println(gz, 0);
      return true;
    }
    return false;
  }

  // ----- 2. 「最初の直線」フォールバック -----
  // 検出開始から STRAIGHT_INITIAL_FORCE_MS 経過しても、まだ何もセグメントが
  // 確定しておらず、かつカーブイベントにも入っていない場合は、強制的に直線確定する。
  if (currentSegment == SEG_NONE
      && !inCurveEvent
      && (now - detectionStartMs) > STRAIGHT_INITIAL_FORCE_MS) {
    currentSegment = SEG_STRAIGHT;
    segmentCandidate = SEG_STRAIGHT;
    return true;
  }

  // ----- 3. カーブイベントの管理 -----
  // (3a) カーブ活性度の追跡
  if (absGz > STRAIGHT_TH) {
    lastHighActivityMs = now;
    // カーブイベント開始判定: |gz| > CURVE_TH_LOW で開始
    if (!inCurveEvent && absGz >= CURVE_TH_LOW) {
      inCurveEvent = true;
      continuousCurveAccum = 0.0f;
    }
  }

  // (3b) カーブイベント中は累積積分
  if (inCurveEvent) {
    continuousCurveAccum += gz * dtSec;
    // 表示用候補 (BLE notify): 累積の符号で暫定方向
    segmentCandidate = (continuousCurveAccum < 0) ? SEG_RIGHT_90 : SEG_LEFT_90;
  } else {
    segmentCandidate = (absGz < STRAIGHT_TH) ? SEG_STRAIGHT : SEG_NONE;
  }

  uint32_t timeSinceActivity = now - lastHighActivityMs;

  // (3c) カーブイベント終了判定
  if (inCurveEvent && timeSinceActivity > SEG_CURVE_END_MS) {
    // 累積角度から最終セグメントを決定
    uint8_t finalSeg = classifyAccum(continuousCurveAccum);
    // デバッグ表示用に確定直前の値を退避
    lastCommitAccum = continuousCurveAccum;
    bool changed = false;
    if (finalSeg != SEG_NONE && finalSeg != currentSegment) {
      currentSegment = finalSeg;
      changed = true;
    }
    // カーブイベント終了
    inCurveEvent = false;
    continuousCurveAccum = 0.0f;
    return changed;
  }

  // ----- 4. 直進確定判定 -----
  // カーブが終わってさらに静かな状態が STRAIGHT_MIN_MS 続いたら STRAIGHT
  if (!inCurveEvent && timeSinceActivity > STRAIGHT_MIN_MS) {
    if (currentSegment != SEG_STRAIGHT) {
      currentSegment = SEG_STRAIGHT;
      return true;
    }
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
    }
  } else {
    // 期待と違う場合
    // 直進は許容、index 維持 (パターン中の経由扱い)
    if (currentSegment == SEG_STRAIGHT && patternMatchIdx > 0) {
      // index 維持
      return;
    }
    // それ以外は進捗を 0 にリセットしつつ、現在のセグメントが1手目に該当するか即判定
    patternMatchIdx = 0;
    if (segmentMatches(userPattern[0], currentSegment)) {
      patternMatchIdx = 1;
      if (patternMatchIdx >= userPatternLen) {
        executeAction(now);
        patternMatchIdx = 0;
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
// ★新規: ジャイロデータ + 内部状態を 50Hz で配信 (10 bytes)
//   [0..1] gz_x10 (int16, dps × 10)
//   [2..3] continuousCurveAccum (int16, deg, 符号付き: 正=左回転、負=右回転)
//   [4..5] maxGzAmplitudeRecent_x10 (uint16, dps × 10)
//   [6]    zeroCrossIdx (uint8, ウェーブ判定中のゼロクロス数)
//   [7]    segmentCandidate (uint8, 現在の暫定候補セグメントID)
//   [8..9] peakSignedAccum (int16, deg, |累積| のピーク値、符号付き)
BLECharacteristic gyroChar(
  "12345678-1234-5678-1234-56789abcdefb",
  BLERead | BLENotify,
  10
);

// ★新規: 高解像度バルクダンプ characteristic
// 走行中は内部のリングバッファに 200Hz で gz サンプルを蓄積。
// 「停止」時 or ホストからの dump 要求時に、ここから連続notify でデータを送る。
//
// パケット構造 (最大 21 bytes):
//   [0..1]  seqNum (uint16, 0始まり、最後のパケットは 0xFFFF)
//   [2]     count  (uint8,  このパケットに含むサンプル数 0~9)
//   [3..]   gz_x10 のリスト (int16 × count, 最大9個=18bytes)
//
// 「停止」時にデバイスから自動的に dump がフラッシュされる。
// ホスト側からの再要求は commandChar に 5 (CMD_BULK_REQUEST) を書くと再送可能。
BLECharacteristic bulkChar(
  "12345678-1234-5678-1234-56789abcdefc",
  BLERead | BLENotify,
  21
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

// ============================================================
// バルクバッファ操作 (高解像度ジャイロログ)
// ============================================================
static inline void bulkBufReset() {
  bulkBufWriteIdx = 0;
  bulkBufCount = 0;
  bulkOverflowed = false;
}

static inline void bulkBufAppend(float gzDps) {
  int32_t v = (int32_t)(gzDps * 10.0f);
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  bulkBuf[bulkBufWriteIdx++] = (int16_t)v;
  if (bulkBufWriteIdx >= BULK_BUF_LEN) {
    bulkBufWriteIdx = 0;
    bulkOverflowed = true;
  }
  if (bulkBufCount < BULK_BUF_LEN) bulkBufCount++;
}

// ダンプ送信を開始
//   呼ぶ前提: 走行が停止していること
static void bulkSendStart() {
  if (bulkBufCount == 0) return;
  bulkSendActive = true;
  bulkSendSeq = 0;
  // オーバーフローしていれば、書き込みヘッド次の位置から読み出す(古いデータが既に上書きされている)
  if (bulkOverflowed) {
    bulkSendIdx = bulkBufWriteIdx;
  } else {
    bulkSendIdx = 0;
  }
  bulkSendLastMs = 0;
  uint32_t pkts = (bulkBufCount + BULK_SAMPLES_PER_PACKET - 1) / BULK_SAMPLES_PER_PACKET;
  uint32_t estMs = pkts * BULK_SEND_INTERVAL_MS;
  Serial.print("[BULK] send start, count=");
  Serial.print(bulkBufCount);
  Serial.print(" packets=");
  Serial.print(pkts);
  Serial.print(" est=");
  Serial.print(estMs);
  Serial.println("ms");
}

// 送信完了マーカーを送る
static void bulkSendEndMarker() {
  uint8_t pkt[3];
  pkt[0] = 0xFF;  // seq = 0xFFFF (終端)
  pkt[1] = 0xFF;
  pkt[2] = 0;     // count = 0
  bulkChar.writeValue(pkt, 3);
  Serial.println("[BULK] end marker sent");
}

// loop() から呼ばれる送信ステップ
static void bulkSendStep(uint32_t now) {
  if (!bulkSendActive) return;
  if ((now - bulkSendLastMs) < BULK_SEND_INTERVAL_MS) return;
  bulkSendLastMs = now;

  uint8_t pkt[3 + BULK_SAMPLES_PER_PACKET * 2];
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < BULK_SAMPLES_PER_PACKET; i++) {
    if (bulkSendSeq * BULK_SAMPLES_PER_PACKET + cnt >= bulkBufCount) break;
    int16_t v = bulkBuf[bulkSendIdx];
    bulkSendIdx++;
    if (bulkSendIdx >= BULK_BUF_LEN) bulkSendIdx = 0;
    pkt[3 + cnt * 2]     = (uint8_t)(v & 0xFF);
    pkt[3 + cnt * 2 + 1] = (uint8_t)((v >> 8) & 0xFF);
    cnt++;
  }

  if (cnt == 0) {
    // 全データ送信完了
    bulkSendEndMarker();
    bulkSendActive = false;
    return;
  }

  pkt[0] = (uint8_t)(bulkSendSeq & 0xFF);
  pkt[1] = (uint8_t)((bulkSendSeq >> 8) & 0xFF);
  pkt[2] = cnt;
  bulkChar.writeValue(pkt, 3 + cnt * 2);
  // 100 パケットごとに進捗ログ
  if ((bulkSendSeq % 100) == 0 && bulkSendSeq > 0) {
    Serial.print("[BULK] sent ");
    Serial.print(bulkSendSeq);
    Serial.println(" packets");
  }
  bulkSendSeq++;
}


static void onCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t cmd = commandChar.value();
  uint32_t now = millis();
  if (cmd == 0) {
    // 停止: モーターブレーキ + バルクダンプ開始
    isRunning = false;
    motorBrake();
    // ★ 停止時に進行中のカーブイベントを強制確定する
    //   (短い走行で停止が早く押されると、本来180度カーブだったのに
    //    確定される前に走行終了してしまうのを防ぐ)
    if (inCurveEvent && fabsf(continuousCurveAccum) >= ACCUM_ANGLE_90_MIN) {
      uint8_t finalSeg = classifyAccum(continuousCurveAccum);
      lastCommitAccum = continuousCurveAccum;
      if (finalSeg != SEG_NONE && finalSeg != currentSegment) {
        currentSegment = finalSeg;
        currentSegChar.writeValue(currentSegment);
        Serial.print("[STOP-COMMIT] ");
        const char* nm = "?";
        switch (currentSegment) {
          case SEG_RIGHT_90:  nm = "RIGHT_90"; break;
          case SEG_RIGHT_180: nm = "RIGHT_180"; break;
          case SEG_LEFT_90:   nm = "LEFT_90"; break;
          case SEG_LEFT_180:  nm = "LEFT_180"; break;
          default:            nm = "?"; break;
        }
        Serial.print(nm);
        Serial.print(" accum=");
        Serial.println(continuousCurveAccum, 1);
      }
      inCurveEvent = false;
      continuousCurveAccum = 0.0f;
    }
    if (bulkBufCount > 0 && !bulkSendActive) {
      bulkSendStart();
    }
  } else if (cmd == 1) {
    // ★走行開始: モーターを必ず 0 から開始 + 検出状態を完全にリセット
    motorDuty(0);
    // ★ ジャイロバイアスを再キャリブレーション
    //   起動時のキャリブレーションだけだと、温度変化やマシンの姿勢変化で
    //   実走行時に大きなオフセットが残る (実機計測で +80~120dps の例あり)。
    //   走行開始時、まだモーターを回す前の静止状態で再取得して、
    //   今回の走行に最適なバイアスを使う。
    //   キャリブレーション中は ~100ms ブロッキングするが、その間は
    //   ソフトスタート前なので問題なし。
    {
      float sum = 0.0f;
      const uint8_t N = 50;
      for (uint8_t i = 0; i < N; i++) {
        sum += myIMU.readFloatGyroZ();
        delay(2);
      }
      gzBias = sum / N;
      Serial.print("[CMD] gzBias recalibrated: ");
      Serial.println(gzBias, 2);
      // フィルタも初期化
      for (uint8_t i = 0; i < GZ_FILT_N; i++) gzBuf[i] = 0;
    }
    isRunning = true;
    bootStartMs = millis();  // キャリブレーションで時間が経ったので now を更新
    softStartDone = false;
    runMode = MODE_NORMAL;
    patternMatchIdx = 0;
    resetDetectionState(bootStartMs);
    patternMatchedFlag = false;
    // バルクバッファもリセット (新規セッション)
    bulkSendActive = false;
    bulkBufReset();

    // ★Web側にも即座に状態リセットを通知
    matchedFlagChar.writeValue(0);
    matchIdxChar.writeValue(0);
    currentSegChar.writeValue(0);

    Serial.println("[CMD] start running");
  } else if (cmd == 2) {
    // リセット: 停止扱い + バッファクリア (ダンプはしない)
    isRunning = false;
    motorBrake();
    patternMatchIdx = 0;
    patternMatchedFlag = false;
    bulkSendActive = false;
    bulkBufReset();
    matchedFlagChar.writeValue(0);
    matchIdxChar.writeValue(0);
    currentSegChar.writeValue(0);
    Serial.println("[CMD] reset");
  } else if (cmd == 5) {
    // バルクダンプ再要求
    if (bulkBufCount > 0 && !bulkSendActive) {
      bulkSendStart();
    } else {
      Serial.println("[CMD] bulk request: nothing to send");
    }
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
  configService.addCharacteristic(gyroChar);
  configService.addCharacteristic(bulkChar);

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

  uint8_t emptyGyro[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  gyroChar.writeValue(emptyGyro, 10);
  // バルクは空ステート (3 バイト終端マーカー)
  uint8_t emptyBulk[3] = {0xFF, 0xFF, 0};
  bulkChar.writeValue(emptyBulk, 3);
  bulkBufReset();

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
    // ★走行停止中もバルクダンプ送信は進める (停止後にホストへ転送するため)
    bulkSendStep(now);
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

  // ★ 動的バイアス追跡 (ドリフト補正)
  //   走行中、明確に「直線セグメント」かつ「カーブイベント外」かつ「|gz|が小さい」時、
  //   gzBias を非常にゆっくり更新する。
  //   これにより、温度変化やマシンの姿勢変化による緩やかなドリフトを追跡できる。
  //   更新は超低速 (時定数 ~10秒) で、急峻な変化は無視するためのガード付き。
  //   静止時 gz の値は実機計測で +80~120dps の例があり、放置すると累積に
  //   1秒で 100° 近い偽の角度が積まれてしまうため、これは必須。
  if (isRunning && currentSegment == SEG_STRAIGHT && !inCurveEvent
      && fabsf(gzFilt) < 30.0f) {
    // gzBias は補正後の gz に対する追加オフセット
    // 1tick (5ms) ごとに 0.05% だけ実測 gzRaw に近づける = 時定数 ~10秒
    gzBias += (gzRaw - gzBias) * 0.0005f;
  }

  // 高解像度バッファに記録 (走行中のみ、200Hz 相当)
  // 走行終了後にホスト側で解析できるよう、フィルタ後の gz を順次格納
  if (isRunning) {
    bulkBufAppend(gzFilt);
  }

  // ウェーブ判定は updateSegmentTransition の中で行うので、ここでは呼ばない

  bool changed = updateSegmentTransition(gzFilt, now);
  if (changed) {
    // 同一値の連続 writeValue を抑制 (BLE 側の二重 dispatch 対策)
    static uint8_t lastWrittenSeg = 0xFF;
    if (currentSegment != lastWrittenSeg) {
      lastWrittenSeg = currentSegment;
      currentSegChar.writeValue(currentSegment);
    }
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
    // カーブ確定時の累積角度 (実機チューニング用に重要)
    Serial.print(" lastAccum=");
    Serial.print(lastCommitAccum, 1);
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
          // パターン完了 → 自動的にバルクダンプ開始 (高解像度ログをホストへ転送)
          if (bulkBufCount > 0 && !bulkSendActive) {
            bulkSendStart();
          }
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

  // ジャイロライブデータと statusChar を高頻度配信 (20ms 周期 ≒ 50Hz)
  // メインループは 200Hz で回るので、ここで間引いて BLE 送信負荷を抑える。
  // 50Hz は Web Bluetooth Notify の上限近くだが、安定動作する。
  if ((now - lastNotifyMs) >= 20) {
    lastNotifyMs = now;
    statusChar.writeValue((isRunning ? 1 : 0) | (runMode << 1));
    int16_t gzx10 = (int16_t)(gzFilt * 10.0f);
    if (gzFilt >  3276.7f) gzx10 =  32767;
    if (gzFilt < -3276.8f) gzx10 = -32768;
    int16_t accumDeg = (int16_t)continuousCurveAccum;
    if (continuousCurveAccum >  32767.0f) accumDeg =  32767;
    if (continuousCurveAccum < -32768.0f) accumDeg = -32768;
    // 旧フィールド (max_amp, zero_cross, peak) はシンプル設計では使わないので 0 を送る
    uint16_t ampx10 = 0;
    int16_t peakDeg = 0;

    uint8_t gyroBuf[10];
    gyroBuf[0] = (uint8_t)(gzx10 & 0xFF);
    gyroBuf[1] = (uint8_t)((gzx10 >> 8) & 0xFF);
    gyroBuf[2] = (uint8_t)(accumDeg & 0xFF);
    gyroBuf[3] = (uint8_t)((accumDeg >> 8) & 0xFF);
    gyroBuf[4] = (uint8_t)(ampx10 & 0xFF);
    gyroBuf[5] = (uint8_t)((ampx10 >> 8) & 0xFF);
    gyroBuf[6] = 0;  // 旧 zeroCrossIdx
    gyroBuf[7] = segmentCandidate;
    gyroBuf[8] = (uint8_t)(peakDeg & 0xFF);
    gyroBuf[9] = (uint8_t)((peakDeg >> 8) & 0xFF);
    gyroChar.writeValue(gyroBuf, 10);
  }

  // バルクダンプ送信進行 (走行停止後のオフライン転送)
  bulkSendStep(now);
}
