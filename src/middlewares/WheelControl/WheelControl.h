#pragma once

#include "mbed.h"

#include "MotorSpeed.h"
#include "QEI.h"
#include "WheelMotor.h"
#include "WheelPID.h"

#define WHEELCONTROL_THREAD_PRIORITY osPriorityHigh
#define WHEELCONTROL_THREAD_STACK_SIZE 1024
#define WHEELCONTROL_THREAD_NAME "WHEELCONTROL"

#define WHEELCONTROL_PERIOD 50ms

enum DIRECTION { FOWARD, REVERSE };

class WheelControl {
public:
  explicit WheelControl(WheelMotor *wheelmotor, WheelPID *wheelpid, MotorSpeed *motorSpeed);
  void setTargetSpeed(double speed);
  void setDirection(DIRECTION direction);
  void start();
  void stop();

private:
  void updateSensorSpeed();
  void threadLoop();
  DIRECTION _direction;
  double _output = 0.0;
  double _sensorSpeed = 0.0;
  double _targetSpeed = 0.0;
  WheelMotor *_wheelmotor;
  MotorSpeed *_motorSpeed;
  WheelPID *_wheelpid;
  PwmOut *_in1;
  PwmOut *_in2;
  unique_ptr<Thread> _thread;
};