#pragma once

#include "mbed.h"
#include <cstdint>

#include "WheelControl.h"
#include "localization.h"
#include "TorqueControl.h"

#define NAVIGATION_THREAD_PRIORITY osPriorityAboveNormal
#define NAVIGATION_THREAD_STACK_SIZE 1024
#define NAVIGATION_THREAD_NAME "NAVIGATION"

#define NAVIGATION_PERIOD 200ms

class Navigation {
public:
    explicit Navigation(Localization* localization, WheelControl* leftWheelControl, WheelControl* rightWheelControl, TorqueControl* torqueControl);
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
  Localization *_localization;
  double _targetX = 0.0;
  double _targetY = 0.0;
  double _eps = 0.0;

private:
    void threadLoop();
    const double _gainKL = 17.0;//1mのずれで2RPMの差
    const double _gainKT = 8.0;
    double _cruiseSpeed = 5.0; // rpm
    double _deltaV = 0.0;
    double _leftTargetSpeed = 0.0;
    double _rightTargetSpeed = 0.0;
    WheelControl* _leftWheelControl;
    WheelControl* _rightWheelControl;
    TorqueControl* _torqueControl;
    unique_ptr<Thread> _thread;
};