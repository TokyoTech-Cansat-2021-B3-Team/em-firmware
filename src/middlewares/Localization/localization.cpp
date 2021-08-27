#include "localization.h"

#include "fusion-odometry.h"
#include "mbed.h"
#include <memory>

Localization::Localization(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, LSM9DS1 *imu, FusionOdometry *ekf,
                           double wheelDistance, double wheelRadius)
    : _leftMotorSpeed(leftMotorSpeed), _rightMotorSpeed(rightMotorSpeed), _wheelDistance(wheelDistance),
      _wheelRadius(wheelRadius), _imu(imu), _ekf(ekf), _timer(Timer()), _thread() {}

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
  _timer.start();
  while (true) {
    double gyrZ_rps = _imu->gyrY() * PI / 180.0;
    double z[] = {getAngularVelocityFromWheelOdometry(), gyrZ_rps, getVelocityFromWheelOdometry()};
    long now = _timer.elapsed_time().count();
    double dt = (_previousTime - now) * 1.0e-6;
    _previousTime = now;
    _ekf->step_with_updateQR(z, dt);
    _theta = _ekf->getX(0);
    _xpk = _ekf->getX(3);
    _ypk = _ekf->getX(4);
    _vpk = _ekf->getX(5);
    _omega_zk = _ekf->getX(1);
    ThisThread::sleep_for(LOCALIZATION_PERIOD);
  }
}

double Localization::getAngularVelocityFromWheelOdometry() {
  return (-getVelocityLeft() + getVelocityRight()) / _wheelDistance;
}

double Localization::getVelocityFromWheelOdometry() {
  return (getVelocityLeft() + getVelocityRight()) / 2.0;
}

double Localization::getVelocityLeft() {
  return -1 * _leftMotorSpeed->currentSpeedRPS() * _wheelRadius;
}

double Localization::getVelocityRight() {
  return -1 * _rightMotorSpeed->currentSpeedRPS() * _wheelRadius;
}

double Localization::x() {
  return _xpk;
}

double Localization::y() {
  return _ypk;
}

double Localization::v() {
  return _vpk;
}

double Localization::theta() {
  return _theta;
}

double Localization::omega() {
  return _omega_zk;
}