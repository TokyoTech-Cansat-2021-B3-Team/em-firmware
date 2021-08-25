#include "WheelMotor.h"

WheelMotor::WheelMotor(PwmOut *in1, PwmOut *in2) : DCMotor(in1, in2, WHEEL_MOTOR_PWM_PERIOD), _direction(FOWARD) {
  if (_in2 != nullptr) {
    *_in2 = 0.0;
    _in2->suspend();
  }
  if (_in1 != nullptr) {
    _in1->period_us(_periodUs);
    *_in1 = 0.0;
  }
}

void WheelMotor::forward(float duty) {
  if (_direction == FOWARD) {
    if (_in1 != nullptr) *_in1 = duty;
  } else {
    if (_in2 != nullptr) {
      *_in2 = 0.0;
      _in2->suspend();
    }
    if (_in1 != nullptr) {
      _in1->resume();
      _in1->period_us(_periodUs);
      *_in1 = duty;
    }
  }
}

void WheelMotor::reverse(float duty) {
  if (_direction == REVERSE) {
    if (_in2 != nullptr) *_in2 = duty;
  } else {
    if (_in1 != nullptr) {
      *_in1 = 0.0;
      _in1->suspend();
    }
    if (_in2 != nullptr) {
      _in2->resume();

      _in2->period_us(_periodUs);
      *_in1 = duty;
    }
  }
}

void WheelMotor::stop() {
  if (_in1 != nullptr) {
    *_in1 = 0.0;
  }

  if (_in2 != nullptr) {
    *_in2 = 0.0;
  }
}
