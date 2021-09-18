#pragma once

#include "mbed.h"
#include <cstdint>

#include "MotorSpeed.h"
#include "fusion-odometry.h"
#include "localization.h"
#include "lsm9ds1.h"

#define LOCALIZATION_THREAD_PRIORITY osPriorityHigh
#define LOCALIZATION_THREAD_STACK_SIZE 1024
#define LOCALIZATION_THREAD_NAME "LOCALIZATION"

class EKFLocalization : public Localization {
public:
  explicit EKFLocalization(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, LSM9DS1 *imu, FusionOdometry *ekf,
                           double wheelDistance, double wheelRadius);

private:
  void threadLoop();
  FusionOdometry *_ekf;
  double _stateVector[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
};