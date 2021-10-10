#include "Stepper.h"

Stepper::Stepper(DigitalOut *step, DigitalOut *enable)
    : _step(step),     //
      _enable(enable), //
      _count(0),       //
      _target(0),      //
      _flag(false)     //
{}

int Stepper::getStep(double angle) const {
  // degrees to step
  return static_cast<int>(angle / STEPPER_STEP_ANGLE);
}

chrono::microseconds Stepper::getPeriodUs(double speed) const {
  // rps to step
  chrono::microseconds ret(static_cast<int>(1000.0 * 1000.0 / speed / STEPPER_STEP_PER_ROTATION));

  return ret;
}

void Stepper::stepTickerCallback() {
  *_step = !(*_step);

  _count++;

  if (_count > _target) {
    _flag = true;
    _stepTicker.detach();
  }
}

void Stepper::rotate(double angle, double speed) {
  _count = 0;
  _target = getStep(angle);
  _flag = false;

  bool tmpEnable = static_cast<bool>(*_enable);

  if (tmpEnable) {
    *_enable = 0;
  }

  _stepTicker.attach(callback(this, &Stepper::stepTickerCallback), getPeriodUs(speed));

  while (!_flag) {
    ThisThread::sleep_for(STEPPER_TICKER_WAIT_INTERVAL);
  }

  if (tmpEnable) {
    *_enable = 1;
  }
}

void Stepper::idleCurrent(bool enable) {
  *_enable = static_cast<int>(!enable);
}
