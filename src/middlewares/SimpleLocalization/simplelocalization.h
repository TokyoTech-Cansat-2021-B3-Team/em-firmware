#pragma once

#include "mbed.h"
#include <cstdint>

#include "localization.h"
#include "MotorSpeed.h"
#include "lsm9ds1.h"

#define SIMPLELOCALIZATION_THREAD_PRIORITY osPriorityAboveNormal
#define SIMPLELOCALIZATION_THREAD_STACK_SIZE 1024
#define SIMPLELOCALIZATION_THREAD_NAME "SIMPLELOCALIZATION"

#define SIMPLELOCALIZATION_PERIOD LOCALIZATION_PERIOD
#define ODOMETRY_PERIOD SIMPLELOCALIZATION_PERIOD

class SimpleLocalization : protected Localization {
public:
  explicit SimpleLocalization(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, double wheelDistance,
                              double wheelRadius);
  void start();
  void stop();

private:
  void threadLoop();
  double _dt = 0.0;
  double _previousTime = 0;
  Timer _timer;
  MotorSpeed *_leftMotorSpeed;
  MotorSpeed *_rightMotorSpeed;
  unique_ptr<Thread> _thread;
};