#pragma once

#include "mbed.h"
#include <cstdint>

#include "MotorSpeed.h"
#include "lsm9ds1.h"

#define LOCALIZATION_THREAD_PRIORITY osPriorityAboveNormal
#define LOCALIZATION_THREAD_STACK_SIZE 1024
#define LOCALIZATION_THREAD_NAME "LOCALIZATION"

#define LOCALIZATION_PERIOD 50ms

class Localization {
public:
  explicit Localization(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, double wheelDistance,
                        double wheelRadius);
  void start();
  void stop();
  double x();
  double y();
  double v();
  double theta();
  double omega();
  double getAngularVelocityFromWheelOdometry();
  double getVelocityFromWheelOdometry();

protected:
  virtual void threadLoop();
  double getVelocityLeft();
  double getVelocityRight();
  const double _wheelDistance;
  const double _wheelRadius;
  const double PI = 3.141592653589793;
  double _theta = 0.0;
  double _omega_z = 0.0;
  double _x = 0.0;
  double _y = 0.0;
  double _v = 0.0;
  MotorSpeed *_leftMotorSpeed;
  MotorSpeed *_rightMotorSpeed;
  unique_ptr<Thread> _thread;
};
