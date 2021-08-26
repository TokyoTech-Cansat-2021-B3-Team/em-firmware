#include "SimpleLocalization.h"

#include "fusion-odometry.h"
#include "mbed.h"
#include <memory>

SimpleLocalization::SimpleLocalization(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, double wheelDistance,
                                       double wheelRadius)
    : _leftMotorSpeed(leftMotorSpeed), _rightMotorSpeed(rightMotorSpeed), _wheelDistance(wheelDistance),
      _wheelRadius(wheelRadius), _thread() {}

void SimpleLocalization::start() {
  _thread = make_unique<Thread>(SIMPLELOCALIZATION_THREAD_PRIORITY, SIMPLELOCALIZATION_THREAD_STACK_SIZE, nullptr,
                                SIMPLELOCALIZATION_THREAD_NAME);
  _thread->start(callback(this, &SimpleLocalization::threadLoop));
}

void SimpleLocalization::stop() {
  _thread->terminate();
  _thread.reset();
}

void SimpleLocalization::threadLoop() {
  while (true) {
    _dt = static_cast<std::chrono::duration<double>>(ODOMETRY_PERIOD).count();
    _omega = getAngularVelocityFromWheelOdometry();
    _theta += _omega * _dt;
    _v = getVelocityFromWheelOdometry();
    _x += _v * _dt * cos(_theta);
    _y += _v * _dt * sin(_theta);
    ThisThread::sleep_for(SIMPLELOCALIZATION_PERIOD);
  }
}

double SimpleLocalization::getAngularVelocityFromWheelOdometry() {
  return (-getVelocityLeft() + getVelocityRight()) / _wheelDistance;
}

double SimpleLocalization::getVelocityFromWheelOdometry() {
  return (getVelocityLeft() + getVelocityRight()) / 2.0;
}

double SimpleLocalization::getVelocityLeft() {
  return _leftMotorSpeed->currentSpeedRPS() * _wheelRadius;
}

double SimpleLocalization::getVelocityRight() {
  return _rightMotorSpeed->currentSpeedRPS() * _wheelRadius;
}

double SimpleLocalization::x() {
  return _x;
}

double SimpleLocalization::y() {
  return _y;
}

double SimpleLocalization::theta() {
  return _theta;
}