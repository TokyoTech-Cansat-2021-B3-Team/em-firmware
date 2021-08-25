#pragma once

#include "mbed.h"

#include "Console.h"
#include "Variometer.h"

#define LANDING_SEQUENCE_THREAD_PRIORITY osPriorityNormal
#define LANDING_SEQUENCE_THREAD_STACK_SIZE 1024
#define LANDING_SEQUENCE_THREAD_NAME "LandingSequence"

#define LANDING_SEQUENCE_POLLING_PERIOD VARIOMETER_PERIOD // 垂直速度確認の周期
#define LANDING_SEQUENCE_FALLING_THRESHOLD -1.0           // 落下判定の垂直速度の閾値 -1.0 m/s
#define LANDING_SEQUENCE_LANDING_DURATION 10s             // 静止判定の持続時間 10s
#define LANDING_SEQUENCE_FALLING_DURATION 1s              // 落下判定の持続時間 1s

class LandingSequence {
private:
public:
  using LandingSequenceState = enum {
    Running,
    WaitFalling,
    WaitLanding,
    Complete,
  };

private:
  unique_ptr<Thread> _thread;
  Variometer *_variometer;
  Console *_console;

  LandingSequenceState _state;

public:
private:
  void threadLoop();

  bool isFalling();

  void waitFalling();

  void waitLanding();

public:
  explicit LandingSequence(Variometer *variometer, Console *console);

  void start();

  void stop();

  LandingSequenceState state();
};
