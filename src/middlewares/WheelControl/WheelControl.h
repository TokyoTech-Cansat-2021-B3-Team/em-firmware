#pragma once

#include "mbed.h"
#include <cstdint>

#include "WheelMotor.h"
#include "QEI.h"
#include "wheelPID.h"

#define WHEELCONTROL_THREAD_PRIORITY osPriorityHigh
#define WHEELCONTROL_THREAD_STACK_SIZE 1024
#define WHEELCONTROL_THREAD_NAME "WHEELCONTROL"

#define WHEELCONTROL_PERIOD 10ms

enum DIRECTION{
    FOWARD,
    REVERSE
};

class WheelControl{
public:
    explicit WheelControl(WheelMotor* wheelmotor, WheelPID* wheelpid, QEI* encoder, double gearRatio);
    explicit WheelControl(PwmOut* in1, PwmOut* in2, WheelPID* wheelpid, QEI* encoder, double gearRatio);
    void setTargetSpeed(double speed);
    void setDirection(DIRECTION direction);
    double sensorSpeed();
    void start();
    void stop();
private:
    void updateSensorSpeed();
    void threadLoop();
    double pulsesToRpm(int pulses);
    const double _gearRatio;
    DIRECTION _direction;
    double _output = 0.0;
    double _sensorSpeed = 0.0;
    double _targetSpeed = 0.0;
    WheelMotor* _wheelmotor;
    QEI* _encoder;
    WheelPID* _wheelpid;
    PwmOut* _in1;
    PwmOut* _in2;
    unique_ptr<Thread> _thread;
};