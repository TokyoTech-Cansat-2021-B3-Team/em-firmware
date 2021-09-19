#pragma once

#include "mbed.h"
#include <cstdint>

#include "MotorSpeed.h"
#include "fusion-odometry.h"
#include "localization.h"
#include "lsm9ds1.h"

#define EKFLOCALIZATION_THREAD_PRIORITY osPriorityHigh
#define EKFLOCALIZATION_THREAD_STACK_SIZE 1024
#define EKFLOCALIZATION_THREAD_NAME "LOCALIZATION"

#define EKFLOCALIZATION_PERIOD LOCALIZATION_PERIOD
#define KALMANFILTER_PERIOD EKFLOCALIZATION_PERIOD

class EKFLocalization : public Localization {
public:
  explicit EKFLocalization(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, LSM9DS1 *imu, FusionOdometry *ekf,
                           double wheelDistance, double wheelRadius);
  void start();
  void stop();

private:
  void threadLoop();
  FusionOdometry *_ekf;
  LSM9DS1 *_imu;
  double _stateVector[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
};