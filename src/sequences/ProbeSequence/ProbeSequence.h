#pragma once

#include "mbed.h"

#include "Console.h"
#include "DCMotor.h"
#include "DrillMotor.h"
#include "QEI.h"
#include "Stepper.h"
#include "lsm9ds1.h"

#define PROBE_SEQUENCE_THREAD_PRIORITY osPriorityBelowNormal
#define PROBE_SEQUENCE_THREAD_STACK_SIZE 2048
#define PROBE_SEQUENCE_THREAD_NAME "ProbeSequence"

// パラメータ
#define PROBE_SEQUENCE_VERTICAL_LEAD 0.8     // 上下駆動のねじリード 0.8 mm
#define PROBE_SEQUENCE_VERTICAL_RATIO 249    // 上下駆動のギア比 1:298
#define PROBE_SEQUENCE_VERTICAL_POLLING 10ms // エンコーダの値の確認周期
#define PROBE_SEQUENCE_VERTICAL_TIMEOUT 3min // 上下駆動のタイムアウト時間

// 1本目刺し込み前の初期位置への移動
#define PROBE_SEQUENCE_SETUP_VERTICAL_DUTY 0.3 // 上下Duty
#define PROBE_SEQUENCE_SETUP_LENGTH 0          // 上下駆動下降量 (mm)

// ホルダー回転
#define PROBE_SEQUENCE_HOLDER_ANGLE_FIRST_1 32.0 // 1本目1段階目の回転角
#define PROBE_SEQUENCE_HOLDER_ANGLE_FIRST 15.0   // 1段階目の回転角
#define PROBE_SEQUENCE_HOLDER_ANGLE_SECOND 57.0  // 2段階目の回転角
#define PROBE_SEQUENCE_HOLDER_DELAY 2s           // 1段階目と2段階目の間の時間
#define PROBE_SEQUENCE_HOLDER_SPEED 0.125        // 回転速度 (rps)

// 電極の押し下げ
#define PROBE_SEQUENCE_PROBEPUSH_VERTICAL_DUTY 1.0 // 上下Duty

// 電極接続
#define PROBE_SEQUENCE_CONNECT_DRILL_DUTY 0.3    // ドリルDuty
#define PROBE_SEQUENCE_CONNECT_VERTICAL_DUTY 0.2 // 上下Duty、
#define PROBE_SEQUENCE_CONNECT_LENGTH 10         // 上下駆動下降量 (mm)

// 刺し込み全体
#define PROBE_SEQUENCE_DRILLING_LENGTH 40      // 上下駆動下降量 (mm)、刺しこみ全体
#define PROBE_SEQUENCE_DRILLING_DRILL_DUTY 1.0 // ドリルDuty、sSaG時

// 刺しこみ、r-SaG
#define PROBE_SEQUENCE_DRILLING_VERTICAL_RSAG_DUTY 0.2   // 上下Duty、rSaG時
#define PROBE_SEQUENCE_DRILLING_VERTICAL_RSAG_LENGTH 15  // 上下駆動下降量 (mm)、rSaG分
#define PROBE_SEQUENCE_DRILLING_VERTICAL_RSAG_GOTIME 3   // GoTime (100ms)
#define PROBE_SEQUENCE_DRILLING_VERTICAL_RSAG_STOPTIME 3 // GoTime (ms)

//刺しこみ、s-SaG
#define PROBE_SEQUENCE_DRILLING_VERTICAL_SSAG_DUTY 0.2 // 上下Duty、sSaG時
#define PROBE_SEQUENCE_DRILLING_VERTICAL_SSAG_THRESHOLD_RATIO 0.8
//#define PROBE_SEQUENCE_DRILLING_VERTICAL_SSAG_THRESHOLD 0.12     // 刺しこみ速度閾値 (mm/100ms)
#define PROBE_SEQUENCE_DRILLING_VERTICAL_SSAG_SAMPLINGTIME 1     // サンプリングタイム (100ms)
#define PROBE_SEQUENCE_DRILLING_VERTICAL_SSAG_SENSINGINTARVAL 10 // 刺しこみ速度判定頻度 (100ms)
#define PROBE_SEQUENCE_DRILLING_VERTICAL_SSAG_STOPTIME 30        // 上下駆動停止時間 (100ms)

// 初期位置に戻る
#define PROBE_SEQUENCE_BACK_VERTIVAL_DUTY 1.0 // 上下Duty

class ProbeSequence {
private:
public:
  // 電極の番号
  using ProbeNumber = enum {
    Probe1 = 1,
    Probe2 = 2,
    Probe3 = 3,
    Probe4 = 4,
  };

  using ProbeSequenceState = enum {
    Running,
    Complete,
    SequenceTimeout,
    Stop,
  };

private:
  unique_ptr<Thread> _thread;

  DrillMotor *_drillMotor;
  DCMotor *_verticalMotor;
  Stepper *_loadingMotor;
  QEI *_verticalEncoder;

  Console *_console;

  // 電極の番号をThreadに通知
  ProbeNumber _probeNumber;

  ProbeSequenceState _state;

  bool _isStart;

public:
private:
  void threadLoop();

  void set(double L, double stroke, double duty);

  // ホルダーの回転
  void holderToNext(double first, double second);

  // 電極の押し下げ
  void probepush();

  // 電極の接続
  void connect(double *ptr);

  // 刺し込み
  void drilling(double DrillingVelocity);

  // 初期位置に戻る
  void back();

  // 指定した距離だけ、上下駆動する
  // strokeは下降する向きを正、上昇する向きを負
  void verticalMove(double duty, double L); //刺しこみ時・電極接続時以外の上下駆動

  void verticalMove_connect(double duty, double L,
                            double *ptr); //電極接続時の上下駆動、刺しこみ速度を計測する

  void verticalMove_sSaG(double duty, double L, double DrillingVelocity); // s-SaG

  void verticalMove_rSaG(double duty, double L); // r-SaG

  // 上下駆動の回転数から距離への変換
  double revToLength(int revolution);

  // s-SaG閾値の決定
  double ThresholdDecision(double DrillingVelocity);

public:
  explicit ProbeSequence(DrillMotor *drillMotor, DCMotor *verticalMotor, Stepper *loadingMotor, QEI *verticalEncoder,
                         Console *console);

  void start(ProbeNumber probeNumber);

  void stop();

  ProbeSequenceState state();
};
