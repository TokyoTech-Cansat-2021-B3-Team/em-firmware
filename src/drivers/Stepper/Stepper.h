#pragma once

#include "mbed.h"

class Stepper {
private:
public:
private:
public:
  explicit Stepper();

  // angle: 角度(度), speed: 速度(rps)
  void rotate(double angle, double speed);
};
