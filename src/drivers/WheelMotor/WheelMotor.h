#pragma once

#include "mbed.h"

#include "DCMotor.h"

#define WHEEL_MOTOR_PWM_PERIOD 100us // TODO: PWM周波数調整？

// 車輪用モータ
// チャンネル共有のため、毎度初期化が必要

class WheelMotor : public DCMotor {
private:
public:
private:
  enum DIRECTION { FOWARD, REVERSE };
  DIRECTION _direction;

public:
  explicit WheelMotor(PwmOut *in1, PwmOut *in2);

  // 正転
  void forward(float duty) override;

  // 逆転
  void reverse(float duty) override;

  // 停止
  void stop() override;
};
