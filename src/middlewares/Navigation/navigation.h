#pragma once

#include "mbed.h"
#include <cstdint>

#include "localization.h"
#include "WheelControl.h"

#define NAVIGATION_THREAD_PRIORITY osPriorityHigh
#define NAVIGATION_THREAD_STACK_SIZE 1024
#define NAVIGATION_THREAD_NAME "NAVIGATION"

#define NAVIGATION_PERIOD 200ms

class Navigation{
public:
    explicit Navigation(Localization* localization, WheelControl* leftWheelControl, WheelControl* rightWheelControl);
    void setTargetPosition(double targetX, double targetY, double eps);
    void setCruiseSpeed(double cruiseSpeed);
    void start();
    void stop();
    double leftTargetSpeed();
    double rightTargetSpeed();
    virtual bool checkArrivingTarget();
protected:
    double _y_diff = 0.0;
    double _theta_diff = 0.0;
    double norm(double x, double y);
    virtual void updateDifference();
    Localization* _localization;
    double _targetX = 0.0;
    double _targetY = 0.0;
    double _eps = 0.0;
private:
    void threadLoop();
    const double _gainKL = 0.2;//1mのずれで2RPMの差
    const double _gainKT = 0.4;
    double _cruiseSpeed = 18.0;//rpm
    double _deltaV = 0.0;
    double _leftTargetSpeed = 0.0;
    double _rightTargetSpeed = 0.0;
    WheelControl* _leftWheelControl;
    WheelControl* _rightWheelControl;
    unique_ptr<Thread> _thread;
};