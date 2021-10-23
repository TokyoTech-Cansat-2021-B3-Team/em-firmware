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

#define CALIBRATION_COUNT 1000
#define CALIBRATION_PERIOD 2ms

class EKFLocalization : public Localization {
public:
  explicit EKFLocalization(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, LSM9DS1 *imu, FusionOdometry *ekf,
                           double wheelDistance, double wheelRadius);
  void calibration();
  void start();
  void stop();
  double beta();
  double slip();

private:
  void threadLoop();
  bool _calibratedFlag = false;
  FusionOdometry *_ekf;
  LSM9DS1 *_imu;
  double _beta = 0.0;
  double _slip = 0.0;
  double _stateVector[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
};