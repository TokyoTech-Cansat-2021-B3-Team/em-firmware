#include "DrillMotor.h"

DrillMotor::DrillMotor(PwmOut *in1) : _in1(in1) {}

void DrillMotor::forward(float duty) {
  // PWM周期の設定
  int periodMicroseconds = chrono::duration_cast<chrono::microseconds>(DRILL_MOTOR_PWM_PERIOD).count();
  _in1->period_us(periodMicroseconds);

  *_in1 = duty;
}

void DrillMotor::stop() {
  *_in1 = 0.0;
}
