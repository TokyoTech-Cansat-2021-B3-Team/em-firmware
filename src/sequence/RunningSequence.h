#pragma once

#include "mbed.h"
#include <cstdint>

#include "navigation.h"
#include "localization.h"
#include "MotorSpeed.h"
#include "WheelControl.h"
#include "lsm9ds1.h"


#define RUNNINGSEQUENCE_THREAD_PRIORITY osPriorityHigh
#define RUNNINGSEQUENCE_THREAD_STACK_SIZE 1024
#define RUNNINGSEQUENCE_THREAD_NAME "RUNNINGSEQUENCE"

#define RUNNINGSEQUENCE_PERIOD 100ms

#define RUNNINGSEQUENCE_TERMINATE_TIME 300s

enum RunningSequenceState{
    UNDEFINED,
    WAITING_FIRST_TO_SECOND_POLE,
    MOVING_FIRST_TO_SECOND_POLE,
    ARRIVED_SECOND_POLE,
    WAITING_SECOND_TO_THIRD_POLE,
    MOVING_SECOND_TO_THIRD_POLE,
    ARRIVED_THIRD_POLE,
    WAITING_THIRD_TO_FOURTH_POLE,
    MOVING_THIRD_TO_FOURTH_POLE,
    ARRIVED_FOURTH_POLE,
    TERMINATE
};

enum RunningSqequneceType{
    FIRST,
    SECOND,
    THIRD
};

class RunningSequence{
public:
    explicit RunningSequence(Navigation* navigation, Localization* Localization, LSM9DS1* imu, MotorSpeed* leftMotorSpeed, MotorSpeed* rightMotorSpeed, WheelControl* leftWheelControl, WheelControl* rightWheelControl);
    void start(RunningSqequneceType sequenceType);
    void stop();
    RunningSequenceState state();
    bool isMoving();
    bool isWaiting();
    bool isArrived();
    bool isError();
    int tmp = 0;
private:
    void init();
    void setStatus(RunningSequenceState state);
    void threadLoop();
    void shiftStatusToMovingAndSetTargetPosition();
    void shiftStatusToArrived();
    int _currentStateCount = 0;
    const double _secondPolePosition[2] = {2.0,0.0};
    const double _thirdPolePosition[2] = {4.0,0.0};
    const double _fourthPolePosition[2] = {6.0,0.0};
    const double _secondPoleEPS = 0.1;
    const double _thirdPoleEPS = 0.1;
    const double _fourthPoleEPS = 0.1;
    RunningSequenceState _state;
    Navigation* _navigation;
    Localization* _localization;
    LSM9DS1* _imu;
    MotorSpeed* _leftMotorSpeed;
    MotorSpeed* _rightMotorSpeed;
    WheelControl* _leftWheelControl;
    WheelControl* _rightWheelControl;
    unique_ptr<Thread> _thread;
};