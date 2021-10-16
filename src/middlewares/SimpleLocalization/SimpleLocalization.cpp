#include "simpleLocalization.h"

#include "localization.h"
#include "fusion-odometry.h"
#include "mbed.h"
#include <memory>

SimpleLocalization::SimpleLocalization(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, double wheelDistance,
                                       double wheelRadius)
    : Localization(leftMotorSpeed,rightMotorSpeed, wheelDistance, wheelRadius) {}

void SimpleLocalization::start() {
  _timer.start();
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
    _dt = (_timer.elapsed_time().count() - _previousTime) * 1.0e-6;
    _previousTime = _timer.elapsed_time().count();
    _omega_z = getAngularVelocityFromWheelOdometry();
    _theta += _omega_z * _dt;
    _v = getVelocityFromWheelOdometry();
    _x += _v * _dt * cos(_theta);
    _y += _v * _dt * sin(_theta);
    ThisThread::sleep_for(SIMPLELOCALIZATION_PERIOD);
  }
}