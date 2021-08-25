#pragma once

#include "mbed.h"

#define FUSING_PWM_PERIOD 20ms
#define FUSING_PWM_DUTY 0.5

class Fusing {
private:
  PwmOut *_fuseGate;

public:
private:
public:
  explicit Fusing(PwmOut *fuseGate);

  void heat(chrono::milliseconds durationTime);
};
