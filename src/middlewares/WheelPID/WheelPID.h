#pragma once

#include "mbed.h"

class WheelPID {
public:
  explicit WheelPID();
  void setTargetSpeed(double speed);
  void resetIntegral();
  void updatePIDOutput(double sensorSpeed, chrono::microseconds period);
  double getOutput();

private:
  void updateSensorSpeed();
  double _targetSpeed = 0.0;
  double _sensorSpeed = 0.0;
  double _previousSpeed = 0.0;
  double _output = 0.0;
  double _integral = 0.0;
  double _diff = 0.0;
  double _pGain = 0.01;
  double _iGain = 0.05;
  double _dGain = 0.02;
};
