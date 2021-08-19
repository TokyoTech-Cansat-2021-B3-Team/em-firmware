#pragma once

#include "mbed.h"
#include <cstdint>

#include "navigation.h"

#define RUNNINGSEQUENCE_THREAD_PRIORITY osPriorityHigh
#define RUNNINGSEQUENCE_THREAD_STACK_SIZE 1024
#define RUNNINGSEQUENCE_THREAD_NAME "RUNNINGSEQUENCE"

#define RUNNINGSEQUENCE_PERIOD 1s

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

class RunningSequence{
public:
    explicit RunningSequence(Navigation* navigation);
    void start();
    void stop();
    void setStatus(RunningSequenceState state);
private:
    void threadLoop();
    void shiftStatusToArrived();
    bool isMoving(RunningSequenceState state);
    bool isWaiting(RunningSequenceState state);
    bool isArrived(RunningSequenceState state);
    bool isError(RunningSequenceState state);
    int _currentStateCount = 0;
    const double _secondPolePosition[2] = {10.0,0.0};
    const double _thirdPolePosition[2] = {20.0,0.0};
    const double _fourthPolePosition[2] = {30.0,0.0};
    const double _secondPoleEPS = 0.1;
    const double _thirdPoleEPS = 0.1;
    const double _fourthPoleEPS = 0.1;
    RunningSequenceState _state;
    Navigation* _navigation;
    unique_ptr<Thread> _thread;
};