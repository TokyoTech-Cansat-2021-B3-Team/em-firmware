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
  unique_ptr<Thread> _thread;

  DrillMotor *_drillMotor;
  DCMotor *_verticalMotor;
  Stepper *_loadingMotor;

public:
private:
  void threadLoop();

public:
  explicit ProbeSequence(DrillMotor *drillMotor, DCMotor *verticalMotor, Stepper *loadingMotor);

  void start();

  void stop();
};
