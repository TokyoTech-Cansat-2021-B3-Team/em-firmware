#pragma once

#include "mbed.h"

#include "Console.h"
#include "Variometer.h"

#define LANDING_SEQUENCE_THREAD_PRIORITY osPriorityBelowNormal
#define LANDING_SEQUENCE_THREAD_STACK_SIZE 2048
#define LANDING_SEQUENCE_THREAD_NAME "LandingSequence"

#define LANDING_SEQUENCE_POLLING_PERIOD VARIOMETER_PERIOD // 垂直速度確認の周期
#define LANDING_SEQUENCE_FALLING_THRESHOLD -1.0           // 落下判定の垂直速度の閾値 -1.0 m/s
#define LANDING_SEQUENCE_LANDING_DURATION 30s             // 静止判定の持続時間 30s
#define LANDING_SEQUENCE_FALLING_DURATION 1s              // 落下判定の持続時間 1s

#define LANDING_SEQUENCE_TIMEOUT 1min // シーケンスのタイムアウト時間

class LandingSequence {
private:
public:
  using LandingSequenceState = enum {
    Running,
    WaitFalling,
    WaitLanding,
    Complete,
    SequenceTimeout,
  };

private:
  unique_ptr<Thread> _thread;
  Variometer *_variometer;
  Console *_console;

  LandingSequenceState _state;

  bool _isStart;

public:
private:
  void threadLoop();

  bool isFalling();

  void waitFalling();

  void waitLanding();

  void timeoutCallback();

public:
  explicit LandingSequence(Variometer *variometer, Console *console);

  void start();

  void stop();

  LandingSequenceState state();
};
