#pragma once

#include "mbed.h"
#include <cstdint>

#include "WheelMotor.h"
#include "QEI.h"

#define WHEELPID_THREAD_PRIORITY osPriorityHigh
#define WHEELPID_THREAD_STACK_SIZE 1024
#define WHEELPID_THREAD_NAME "WHEELPID"

#define WHEELPID_PERIOD 200ms
#define WHEELPID_READ_SIZE 255

enum DIRECTION{
    CW,
    CCW
};

class WheelPID{
public:
    explicit WheelPID(WheelMotor* wheelmotor, QEI* qei);
    void setTargetSpeed(double speed);
    void setGearRatio(double gearratio);
    void setDirection(DIRECTION direction);
private:
    void start();
    void stop();
    void threadLoop();
    void updatePIDOutput();
    void updateSensorSpeed();
    double pulsesToRpm(int pulses);
    DIRECTION _direction = CW;
    double _targetSpeed = 0.0;
    double _sensorSpeed = 0.0;
    double _previousSpeed = 0.0;
    double _output = 0.0;
    double _integral = 0.0;
    double _diff = 0.0;
    double _pGain = 0.01;
    double _iGain = 0.05;
    double _dGain = 0.02;
    double _gearRatio = 249.8;
    WheelMotor* _wheelmotor;
    QEI* _qei;
    unique_ptr<Thread> _thread;
};