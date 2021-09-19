#include "localization.h"

#include "fusion-odometry.h"
#include "mbed.h"
#include <memory>

Localization::Localization(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed,  double wheelDistance,
                           double wheelRadius)
    : _leftMotorSpeed(leftMotorSpeed), _rightMotorSpeed(rightMotorSpeed), _wheelDistance(wheelDistance),
      _wheelRadius(wheelRadius), _thread() {}

void Localization::start() {
  _thread = make_unique<Thread>(LOCALIZATION_THREAD_PRIORITY, LOCALIZATION_THREAD_STACK_SIZE, nullptr,
                                LOCALIZATION_THREAD_NAME);
  _thread->start(callback(this, &Localization::threadLoop));
}

void Localization::stop() {
  _thread->terminate();
  _thread.reset();
}

void Localization::threadLoop() {
  
}

double Localization::getAngularVelocityFromWheelOdometry() {
  return (-getVelocityLeft() + getVelocityRight()) / _wheelDistance;
}

double Localization::getVelocityFromWheelOdometry() {
  return (getVelocityLeft() + getVelocityRight()) / 2.0;
}

double Localization::getVelocityLeft() {
  return _leftMotorSpeed->currentSpeedRPS() * _wheelRadius;
}

double Localization::getVelocityRight() {
  return _rightMotorSpeed->currentSpeedRPS() * _wheelRadius;
}

double Localization::x() {
  return _x;
}

double Localization::y() {
  return _y;
}

double Localization::v() {
  return _v;
}

double Localization::theta() {
  return _theta;
}

double Localization::omega() {
  return _omega_z;
}