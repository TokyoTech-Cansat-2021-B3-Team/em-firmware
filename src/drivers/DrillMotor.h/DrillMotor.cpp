#include "DrillMotor.h"

DrillMotor::DrillMotor(DigitalOut *in1) : _in1(in1) {}

void DrillMotor::forward() {
  *_in1 = 1;
}

void DrillMotor::stop() {
  *_in1 = 0;
}
