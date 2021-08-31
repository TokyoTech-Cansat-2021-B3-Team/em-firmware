#pragma once

#include "mbed.h"

// CX28BYJ48
#define STEPPER_STEP_ANGLE (5.625 / 64.0 / (STEPPER_MICROSTEP))
#define STEPPER_STEP_PER_ROTATION (360 / (STEPPER_STEP_ANGLE))
#define STEPPER_MICROSTEP 16 // マイクロステップ: 16

#define STEPPER_TICKER_WAIT_INTERVAL 10ms

class Stepper {
private:
  Ticker _stepTicker;

  DigitalOut *_step;

  // Active Low
  DigitalOut *_enable;

  int _count;
  int _target;
  bool _flag;

public:
private:
  void stepTickerCallback();

  int getStep(double angle) const;

  chrono::microseconds getPeriodUs(double speed) const;

public:
  explicit Stepper(DigitalOut *step, DigitalOut *enable);

  // angle: 角度(度), speed: 速度(rps)
  void rotate(double angle, double speed);

  // 電流 ON/OFF
  // rotateでは強制的にon/offされる
  void idleCurrent(bool enable);
};
