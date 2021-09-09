#include "Fusing.h"
#include <chrono>

Fusing::Fusing(PwmOut *fuseGate) : _fuseGate(fuseGate) {}

void Fusing::heat(chrono::milliseconds durationTime) {
  _fuseGate->period_us(chrono::duration_cast<chrono::microseconds>(FUSING_PWM_PERIOD).count());

  *_fuseGate = FUSING_PWM_DUTY;

  ThisThread::sleep_for(durationTime);

  *_fuseGate = 0.0;
}
