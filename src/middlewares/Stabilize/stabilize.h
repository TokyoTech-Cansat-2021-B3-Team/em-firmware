#pragma once

#include "mbed.h"
#include <cstdint>

#include "WheelMotor.h"
#include "lsm9ds1.h"

#define STABILIZE_THREAD_PRIORITY osPriorityHigh
#define STABILIZE_THREAD_STACK_SIZE 1024
#define STABILIZE_THREAD_NAME "STABILIZE"

#define STABILIZE_PERIOD 1ms

class Stabilize {
public:
  enum STABILIZE_STATE {
    WAITING,
    INVOKE_STABILIZER,
    WAITING_STABILIZER_OPENING,
    PREPARING_INVOKE_ANTI_TOLQUE,
    INVOKE_ANTI_TOLQUE,
    CALM_STABILIZE,
    COMPLETE_STABILIZE
  };
  explicit Stabilize(LSM9DS1 *imu, WheelMotor *leftWheelMotor, WheelMotor *rightWheelMotor);
  void start();
  void stop();
  double currentTheta();
  double currentOutput();
  STABILIZE_STATE state();

private:
  void threadLoop();
  void invokeStabilizer();
  void waitingStabilizerOpening();
  void invokeAntiTolque();
  bool checkStabilizerOpend();
  bool checkCompleteStabilize();
  double getTheta(double accX, double accY, double accZ);
  const double _pGain = 1.0;
  const double _iGain = 0;
  const double _eps = 0.08; // 5度程度
  double _integral = 0.0;
  double _output = 0.0;
  double _theta = 0.0;
  STABILIZE_STATE _state;
  WheelMotor *_leftWheelMotor;
  WheelMotor *_rightWheelMotor;
  LSM9DS1 *_imu;
  unique_ptr<Thread> _thread;
};