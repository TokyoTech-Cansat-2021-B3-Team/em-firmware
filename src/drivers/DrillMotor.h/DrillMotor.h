#pragma once

#include "mbed.h"

// ドリル用モータ
// DRV8833使用
// IN2はGNDに固定

#define DRILL_MOTOR_PWM_PERIOD 20ms // mbed デフォルト

class DrillMotor {
private:
  PwmOut *_in1;

public:
private:
public:
  explicit DrillMotor(PwmOut *in1);

  // ドリルが進む方向に回転
  void forward(float duty);

  // 停止
  void stop();
};
