#pragma once

#include "mbed.h"
#include <cstdint>

#include "MotorSpeed.h"
#include "lsm9ds1.h"

#define SIMPLELOCALIZATION_THREAD_PRIORITY osPriorityAboveNormal
#define SIMPLELOCALIZATION_THREAD_STACK_SIZE 1024
#define SIMPLELOCALIZATION_THREAD_NAME "SIMPLELOCALIZATION"

#define SIMPLELOCALIZATION_PERIOD 50ms
#define ODOMETRY_PERIOD SIMPLELOCALIZATION_PERIOD

class SimpleLocalization {
public:
  explicit SimpleLocalization(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, double wheelDistance,
                              double wheelRadius);
  void start();
  void stop();
  double x();
  double y();
  double theta();

private:
  void threadLoop();
  double getAngularVelocityFromWheelOdometry();
  double getVelocityFromWheelOdometry();
  double getVelocityLeft();
  double getVelocityRight();
  const double _wheelDistance;
  const double _wheelRadius;
  const double PI = 3.141592653589793;
  double _dt = 0.0;
  double _previousTime = 0;
  double _omega = 0.0;
  double _v = 0.0;
  double _theta = 0.0;
  double _x = 0.0;
  double _y = 0.0;
  Timer _timer;
  MotorSpeed *_leftMotorSpeed;
  MotorSpeed *_rightMotorSpeed;
  unique_ptr<Thread> _thread;
};