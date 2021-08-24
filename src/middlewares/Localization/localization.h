#pragma once

#include "mbed.h"
#include <cstdint>

#include "fusion-odometry.h"
#include "MotorSpeed.h"
#include "lsm9ds1.h"

#define LOCALIZATION_THREAD_PRIORITY osPriorityHigh
#define LOCALIZATION_THREAD_STACK_SIZE 1024
#define LOCALIZATION_THREAD_NAME "LOCALIZATION"

#define LOCALIZATION_PERIOD 200ms
#define KALMANFILTER_PERIOD LOCALIZATION_PERIOD

class Localization{
public:
    explicit Localization(MotorSpeed* leftMotorSpeed, MotorSpeed* rightMotorSpeed, LSM9DS1* imu, FusionOdometry* ekf, double wheelDistance, double wheelRadius);
    void start();
    void stop();
    double x();
    double y();
    double v();
    double theta();
    double omega();
    double getAngularVelocityFromWheelOdometry();
    double getVelocityFromWheelOdometry();
private:
    void threadLoop();
    double getVelocityLeft();
    double getVelocityRight();
    const double _wheelDistance;
    const double _wheelRadius;
    const double PI = 3.141592653589793;
    double _theta = 0.0;
    double _omega_zk = 0.0;
    double _xpk = 0.0;
    double _ypk = 0.0;
    double _vpk = 0.0;
    MotorSpeed* _leftMotorSpeed;
    MotorSpeed* _rightMotorSpeed;
    LSM9DS1* _imu;
    FusionOdometry* _ekf;
    unique_ptr<Thread> _thread;
    double _stateVector[5] = {0.0,0.0,0.0,0.0,0.0};
};