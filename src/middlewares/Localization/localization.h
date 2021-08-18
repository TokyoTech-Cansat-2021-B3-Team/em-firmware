#pragma once

#include "mbed.h"
#include <cstdint>

#include "fusion-odometry.h"
#include "WheelControl.h"

#define LOCALIZATION_THREAD_PRIORITY osPriorityHigh
#define LOCALIZATION_THREAD_STACK_SIZE 1024
#define LOCALIZATION_THREAD_NAME "LOCALIZATION"

#define LOCALIZATION_PERIOD 10ms

class Localization{
public:
    explicit Localization(WheelControl* wheelControl, FusionOdometry* ekf);
    void start();
    void stop();
private:
    void threadLoop();
    WheelControl* _wheelControl;
    FusionOdometry* _ekf;
    unique_ptr<Thread> _thread;
};