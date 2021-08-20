#pragma once

#include "mbed.h"
#include <cstdint>

#include "navigation.h"

#define RUNNINGSEQUENCE_THREAD_PRIORITY osPriorityHigh
#define RUNNINGSEQUENCE_THREAD_STACK_SIZE 1024
#define RUNNINGSEQUENCE_THREAD_NAME "RUNNINGSEQUENCE"

#define RUNNINGSEQUENCE_PERIOD 100ms

#define RUNNINGSEQUENCE_TERMINATE_TIME 60s

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
    explicit RunningSequence(Navigation* navigation);
    void start(RunningSqequneceType sequenceType);
    void stop();
    RunningSequenceState state();
    bool isMoving();
    bool isWaiting();
    bool isArrived();
    bool isError();
    int tmp = 0;
private:
    void setStatus(RunningSequenceState state);
    void threadLoop();
    void shiftStatusToMovingAndSetTargetPosition();
    void shiftStatusToArrived();
    int _currentStateCount = 0;
    const double _secondPolePosition[2] = {1.0,0.0};
    const double _thirdPolePosition[2] = {2.0,0.0};
    const double _fourthPolePosition[2] = {3.0,0.0};
    const double _secondPoleEPS = 0.1;
    const double _thirdPoleEPS = 0.1;
    const double _fourthPoleEPS = 0.1;
    RunningSequenceState _state;
    Navigation* _navigation;
    unique_ptr<Thread> _thread;
};