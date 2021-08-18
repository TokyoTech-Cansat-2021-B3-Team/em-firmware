#pragma once

#include "mbed.h"

#include "DCMotor.h"
#include "DrillMotor.h"
#include "Stepper.h"

#define THREAD_PATTERN_THREAD_PRIORITY osPriorityNormal
#define THREAD_PATTERN_THREAD_STACK_SIZE 1024
#define THREAD_PATTERN_THREAD_NAME "ProbeSequence"

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

private:
  unique_ptr<Thread> _thread;

  DrillMotor *_drillMotor;
  DCMotor *_verticalMotor;
  Stepper *_loadingMotor;

  // 電極の番号をThreadに通知
  ProbeNumber _probeNumber;

public:
private:
  void threadLoop();

public:
  explicit ProbeSequence(DrillMotor *drillMotor, DCMotor *verticalMotor, Stepper *loadingMotor);

  void start(ProbeNumber probeNumber);

  void stop();
};
