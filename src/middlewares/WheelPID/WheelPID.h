#pragma once

#include "mbed.h"
#include <cstdint>

class WheelPID {
public:
  explicit WheelPID();
  void setTargetSpeed(double speed);
  void resetIntegral();
  void updatePIDOutput(double sensorSpeed, chrono::microseconds period);
  double getOutput();
  double targetSpeed();

private:
  void updateSensorSpeed();
  double _targetSpeed = 0.0;
  double _previousSpeed = 0.0;
  double _output = 0.0;
  double _integral = 0.0;
  double _diff = 0.0;
  double _pGain = 0.009;
  double _iGain = 0.008;
  double _dGain = 0.00008;
};