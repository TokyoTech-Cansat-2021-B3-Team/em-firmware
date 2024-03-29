#include "ekflocalization.h"
#include "localization.h"

#include "fusion-odometry.h"
#include "mbed.h"
#include <memory>

EKFLocalization::EKFLocalization(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, LSM9DS1 *imu,
                                 FusionOdometry *ekf, double wheelDistance, double wheelRadius)
    : Localization(leftMotorSpeed, rightMotorSpeed, wheelDistance, wheelRadius), _imu(imu), _ekf(ekf) {}

void EKFLocalization::start() {
  _thread = make_unique<Thread>(EKFLOCALIZATION_THREAD_PRIORITY, EKFLOCALIZATION_THREAD_STACK_SIZE, nullptr,
                                EKFLOCALIZATION_THREAD_NAME);
  _thread->start(callback(this, &EKFLocalization::threadLoop));
}

void EKFLocalization::stop() {
  _thread->terminate();
  _thread.reset();
}

void EKFLocalization::threadLoop() {
  while (!_calibratedFlag) {
    ThisThread::sleep_for(EKFLOCALIZATION_PERIOD);
  }
  while (true) {
    double gyrX_rps = _imu->gyrX() * PI / 180.0;
    double z[] = {getAngularVelocityFromWheelOdometry(), gyrX_rps, getVelocityFromWheelOdometry()};
    _ekf->step_with_updateQR(z);
    _theta = _ekf->getX(0);
    _x = _ekf->getX(3);
    _y = _ekf->getX(4);
    _v = _ekf->getX(5);
    _omega_z = _ekf->getX(2);
    _beta = _ekf->getX(1);
    _slip = _ekf->getX(6);
    ThisThread::sleep_for(KALMANFILTER_PERIOD);
  }
}

double EKFLocalization::beta() {
  return _beta;
}

double EKFLocalization::slip() {
  return _slip;
}

void EKFLocalization::calibration() {
  double gyrSum = 0;
  for (int i = 0; i < CALIBRATION_COUNT; i++) {
    gyrSum += _imu->gyrX() * PI / 180.0;
    ThisThread::sleep_for(CALIBRATION_PERIOD);
  }
  _ekf->setGyrBias(gyrSum / (double)CALIBRATION_COUNT);
  _calibratedFlag = true;
}