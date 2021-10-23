#pragma once

#include "mbed.h"
#include <cstdint>

#include "MotorSpeed.h"
#include "QEI.h"
#include "WheelControl.h"
#include "WheelMotor.h"
#include "WheelPID.h"

#define TORQUECONTROL_THREAD_PRIORITY osPriorityAboveNormal
#define TORQUECONTROL_THREAD_STACK_SIZE 1024
#define TORQUECONTROL_THREAD_NAME "TORQUECONTROL"

#define TORQUECONTROL_PERIOD 100ms

class TorqueControl {
public:
  explicit TorqueControl(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, WheelControl *leftWheelControl,
                         WheelControl *rightWheelControl, WheelPID *leftWheelPID, WheelPID *rightWheelPID);
  double cruiseSpeed();
  bool checkNavigatable();
  void setGeneralCruiseSpeed(double speed);
  void setSlowCruiseSpeed(double speed);
  void start();
  void stop();
  enum TORQUESTATE { SLOWSPEED, GENERALSPEED };

private:
  void threadLoop();
  bool checkDecreasingSpeed();
  bool checkSufficientSpeedUp();
  void updateSufficientSpeedUpCount();
  void resetSufficientSpeedUpCount();
  void updateDecreasingSpeedCount();
  void resetDecreasingSpeedCount();
  void updateDeviation();
  void updateDeviationFiltered();
  void setSlowState();
  void setGeneralState();
  const int _sufficientSpeedUpCountThreshold = 7;
  const int _decreasingSpeedCountThreshold = 3;
  const double _slowingDisableDevivationThreshold = 5; // rpm / s
  const double _deviationLowpassFilteredRatio = 0.2;
  double _deviationFilteredLeft = 0.0;
  double _deviationFilteredRight = 0.0;
  double _deviationLeft = 0.0;
  double _deviationRight = 0.0;
  double _sufficientSpeedRatio = 0.7;
  double _slowingDetectSpeedRatio = 0.7;
  double _generalCruiseSpeed = 0.0;
  double _slowCruiseSpeed = 5.0;
  double _leftPreviousSpeed = 0.0;
  double _rightPreviousSpeed = 0.0;
  double _leftSpeedDifference = 0.0;
  double _rightSpeedDifference = 0.0;
  double _cruiseSpeed = 0.0;
  int _sufficientSpeedUpCount = 0;
  int _decreasingSpeedCount = 0;
  TORQUESTATE _state = GENERALSPEED;
  MotorSpeed *_leftMotorSpeed;
  MotorSpeed *_rightMotorSpeed;
  WheelControl *_leftWheelControl;
  WheelControl *_rightWheelControl;
  WheelPID *_leftWheelPID;
  WheelPID *_rightWheelPID;
  unique_ptr<Thread> _thread;
};