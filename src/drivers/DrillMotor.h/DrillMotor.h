#pragma once

#include "mbed.h"

#include "DCMotor.h"

// ドリル用モータ
// IN2はGNDに固定

class DrillMotor : public DCMotor {
private:
public:
private:
  void reverse(float duty) override {}

public:
  // in2をnullptrとする
  // (DCMotorでnullチェックされる)
  // 周期デフォルト
  explicit DrillMotor(PwmOut *in1) : DCMotor(in1, nullptr) {}
};
