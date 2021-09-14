#pragma once

#include "mbed.h"

#include "Console.h"
#include "Logger.h"
#include "stabilize.h"

#define STABILIZESEQUENCE_THREAD_PRIORITY osPriorityBelowNormal
#define STABILIZESEQUENCE_THREAD_STACK_SIZE 2048
#define STABILIZESEQUENCE_THREAD_NAME "STABILIZESEQUENCE"

#define STABILIZESEQUENCE_PERIOD 10ms

#define STABILIZESEQUENCE_TERMINATE_TIME 10s

class StabilizeSequence {
public:
  enum StabilizeSequenceState {
    UNDEFINED,
    WAITING,
    INVOKE_STABILIZER,
    WAITING_STABILIZER_OPENING,
    PREPARING_INVOKE_ANTI_TOLQUE,
    INVOKE_ANTI_TOLQUE,
    CALM_STABILIZE,
    COMPLETE,
    TERMINATE
  };

  explicit StabilizeSequence(Stabilize *stabilize, LSM9DS1 *imu, Console *console, Logger *logger);
  void start();
  void stop();
  StabilizeSequenceState state();
  bool isMoving();
  bool isWaiting();
  bool isArrived();
  bool isError();

private:
  void setStatus(StabilizeSequenceState state);
  void threadLoop();
  void updateStatus();
  chrono::microseconds _previousTime = 0s;
  StabilizeSequenceState _state;
  Stabilize *_stabilize;
  LSM9DS1 *_imu;
  Console *_console;
  Logger *_logger;
  Timer _timer;
  unique_ptr<Thread> _thread;
};
