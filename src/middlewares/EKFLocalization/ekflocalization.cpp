#include "ekflocalization.h"
#include "localization.h"

#include "fusion-odometry.h"
#include "mbed.h"
#include <memory>

EKFLocalization::EKFLocalization(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, LSM9DS1 *imu, FusionOdometry *ekf,
                           double wheelDistance, double wheelRadius)
    : Localization(leftMotorSpeed,rightMotorSpeed,imu,wheelDistance,wheelRadius), _ekf(ekf){}

void EKFLocalization::threadLoop() {
  while (true) {
    double gyrZ_rps = _imu->gyrZ() * PI / 180.0;
    double z[] = {getAngularVelocityFromWheelOdometry(), -gyrZ_rps, getVelocityFromWheelOdometry()};
    _ekf->step_with_updateQR(z);
    _theta = _ekf->getX(0);
    _xpk = _ekf->getX(3);
    _ypk = _ekf->getX(4);
    _vpk = _ekf->getX(5);
    _omega_zk = _ekf->getX(1);
    ThisThread::sleep_for(LOCALIZATION_PERIOD);
  }
}