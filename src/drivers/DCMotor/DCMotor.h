#pragma once

#include "mbed.h"

// DCモータの基底クラス
// DRV8833使用

#define DC_MOTOR_PWM_PERIOD 20ms // mbed デフォルト

class DCMotor {
private:
protected:
  PwmOut *_in1;
  PwmOut *_in2;

  const int _periodUs;
  bool _setPeriod;

public:
private:
public:
  explicit DCMotor(PwmOut *in1, PwmOut *in2, chrono::microseconds period = DC_MOTOR_PWM_PERIOD);

  // 正転
  virtual void forward(float duty);

  // 逆転
  virtual void reverse(float duty);

  // 停止
  virtual void stop();
};
