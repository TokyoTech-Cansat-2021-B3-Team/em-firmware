#include "DCMotor.h"

DCMotor::DCMotor(PwmOut *in1, PwmOut *in2, chrono::microseconds period)
    : _in1(in1),                                                              //
      _in2(in2),                                                              //
      _periodUs(chrono::duration_cast<chrono::microseconds>(period).count()), //
      _setPeriod(false)                                                       //
{}

void DCMotor::forward(float duty) {
  if (_in1 != nullptr) {
    // if (!_setPeriod) {
    _in1->period_us(_periodUs);
    //   _setPeriod = true;
    // }
    *_in1 = duty;
  }

  if (_in2 != nullptr) {
    // if (!_setPeriod) {
    _in2->period_us(_periodUs);
    //   _setPeriod = true;
    // }
    *_in2 = 0.0;
  }
}

void DCMotor::reverse(float duty) {
  if (_in1 != nullptr) {
    // if (!_setPeriod) {
    _in1->period_us(_periodUs);
    //   _setPeriod = true;
    // }
    *_in1 = 0.0;
  }

  if (_in2 != nullptr) {
    // if (!_setPeriod) {
    _in2->period_us(_periodUs);
    //   _setPeriod = true;
    // }
    *_in2 = duty;
  }
}

void DCMotor::stop() {
  if (_in1 != nullptr) {
    *_in1 = 0.0;
  }

  if (_in2 != nullptr) {
    *_in2 = 0.0;
  }
}
