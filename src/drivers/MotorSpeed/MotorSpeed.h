#pragma once

#include "mbed.h"

#include "MissionParameters.h"
#include "QEI.h"

#define MOTORSPEED_THREAD_PRIORITY osPriorityHigh
#define MOTORSPEED_THREAD_STACK_SIZE 1024
#define MOTORSPEED_THREAD_NAME "MOTORSPEED"

#define MOTORSPEED_PERIOD 50ms

class MotorSpeed {
public:
  explicit MotorSpeed(QEI *encoder, double gyarRatio);
  void start();
  void stop();
  double currentSpeedRPM();
  double currentSpeedRPS();

private:
  void threadLoop();
  double pulsesToRpm(int pulses, chrono::microseconds period);
  double rpmToRadPerSecond(double rpm);
  const double _gyarRatio;
  QEI *_encoder;
  double _motorSpeedRPM;
  unique_ptr<Thread> _thread;
};
