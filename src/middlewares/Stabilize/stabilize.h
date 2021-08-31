#pragma once

#include "mbed.h"
#include <cstdint>

#include "WheelMotor.h"
#include "lsm9ds1.h"

#define STABILIZE_THREAD_PRIORITY osPriorityHigh
#define STABILIZE_THREAD_STACK_SIZE 1024
#define STABILIZE_THREAD_NAME "STABILIZE"

#define STABILIZE_PERIOD 1ms

class Stabilize{
public:
    explicit Stabilize(LSM9DS1* imu, WheelMotor* leftWheelMotor, WheelMotor* rightWheelMotor);
    void start();
    void stop();
    double currentTheta();
    double currentOutput();
private:
    void threadLoop();
    double getTheta(double accX, double accY, double accZ);
    const double _pGain = 0.2;
    const double _iGain = 0.01;
    const double _eps = 0.08; // 5度程度
    double _integral = 0.0;
    double _output = 0.0;
    double _theta = 0.0;
    WheelMotor* _leftWheelMotor;
    WheelMotor* _rightWheelMotor;
    LSM9DS1* _imu;
    unique_ptr<Thread> _thread;
};