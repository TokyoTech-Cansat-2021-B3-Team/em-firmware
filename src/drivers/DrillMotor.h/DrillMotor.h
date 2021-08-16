#pragma once

#include "mbed.h"

// ドリル用モータ
// DRV8833使用
// IN2はGNDに固定

class DrillMotor {
private:
  DigitalOut *_in1;

public:
private:
public:
  explicit DrillMotor(DigitalOut *in1);

  // ドリルが進む方向に回転
  void forward();

  // 停止
  void stop();
};
