#pragma once

#include "mbed.h"

#include "DCMotor.h"
#include "DrillMotor.h"
#include "QEI.h"
#include "Stepper.h"

#define PROBE_SEQUENCE_THREAD_PRIORITY osPriorityNormal
#define PROBE_SEQUENCE_THREAD_STACK_SIZE 2048
#define PROBE_SEQUENCE_THREAD_NAME "ProbeSequence"

// パラメータ
#define PROBE_SEQUENCE_VERTICAL_LEAD 0.8     // 上下駆動のねじリード 0.8 mm
#define PROBE_SEQUENCE_VERTICAL_RATIO 249    // 上下駆動のギア比 1:249
#define PROBE_SEQUENCE_VERTICAL_POLLING 10ms // エンコーダの値の確認周期

// 1本目刺し込み前の初期位置への移動
#define PROBE_SEQUENCE_SETUP_VERTICAL_DUTY 0.3 // 上下Duty
#define PROBE_SEQUENCE_SETUP_LENGTH -20        // 上下駆動下降量 (mm)

// ホルダー回転
#define PROBE_SEQUENCE_HOLDER_ANGLE_FIRST 12.0  // 1段階目の回転角
#define PROBE_SEQUENCE_HOLDER_ANGLE_SECOND 60.0 // 2段階目の回転角
#define PROBE_SEQUENCE_HOLDER_DELAY 2s          // 1段階目と2段階目の間の時間
#define PROBE_SEQUENCE_HOLDER_SPEED 0.125       // 回転速度 (rps)

// 電極接続
#define PROBE_SEQUENCE_CONNECT_DRILL_DUTY 0.5    // ドリルDuty
#define PROBE_SEQUENCE_CONNECT_VERTICAL_DUTY 0.5 // 上下Duty
#define PROBE_SEQUENCE_CONNECT_LENGTH 15         // 上下駆動下降量 (mm)

// 刺し込み
#define PROBE_SEQUENCE_DRILLING_DRILL_DUTY 1.0    // ドリルDuty
#define PROBE_SEQUENCE_DRILLING_VERTICAL_DUTY 0.3 // 上下Duty
#define PROBE_SEQUENCE_DRILLING_LENGTH 30         // 上下駆動下降量 (mm)

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

  // 電極の番号をThreadに通知
  ProbeNumber _probeNumber;

  ProbeSequenceState _state;

  bool _isStart;

public:
private:
  void threadLoop();

  void set(double L, double stroke, double duty);

  // ホルダーの回転
  void holderToNext();

  // 電極の接続
  void connect();

  // 刺し込み
  void drilling();

  // 初期位置に戻る
  void back();

  // 指定した距離だけ、上下駆動する
  // strokeは下降する向きを正、上昇する向きを負
  void verticalMove(double duty, double L);

  // 上下駆動の回転数から距離への変換
  double revToLength(int revolution);

public:
  explicit ProbeSequence(DrillMotor *drillMotor, DCMotor *verticalMotor, Stepper *loadingMotor, QEI *verticalEncoder);

  void start(ProbeNumber probeNumber);

  void stop();

  ProbeSequenceState state();
};
