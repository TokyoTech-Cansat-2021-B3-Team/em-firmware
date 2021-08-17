#pragma once

#include "mbed.h"
#include <cstdint>

#include "WheelControl.h"

#define LOCALIZATION_THREAD_PRIORITY osPriorityHigh
#define LOCALIZATION_THREAD_STACK_SIZE 1024
#define LOCALIZATION_THREAD_NAME "LOCALIZATION"

#define LOCALIZATION_PERIOD 10ms

class Localization{
public:
    explicit Localization(WheelMotor* wheelmotor, WheelPID* wheelpid, QEI* encoder, double gearRatio);
    void start();
    void stop();
private:
    void threadLoop();
    unique_ptr<Thread> _thread;
};