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
  while (true) {
    double gyrZ_rps = _imu->gyrZ() * PI / 180.0;
    double z[] = {getAngularVelocityFromWheelOdometry(), -gyrZ_rps, getVelocityFromWheelOdometry()};
    _ekf->step_with_updateQR(z);
    _theta = _ekf->getX(0);
    _x = _ekf->getX(3);
    _y = _ekf->getX(4);
    _v = _ekf->getX(5);
    _omega_z = _ekf->getX(1);
    ThisThread::sleep_for(LOCALIZATION_PERIOD);
  }
}