#include "mbed.h"

#include "PinAssignment.h"

#include "DCMotor.h"
#include "DrillMotor.h"
#include "ProbeSequence.h"
#include "QEI.h"
#include "Stepper.h"

PwmOut motor3In1(M3_IN1);
PwmOut motor3In2(M3_IN2);

PwmOut motor4In1(M4_IN1);

DigitalOut motor5Enable(M5_ENABLE);
DigitalOut motor5Step(M5_STEP);

DCMotor verticalMotor(&motor3In1, &motor3In2);
DrillMotor drillMotor(&motor4In1);
QEI verticalEncoder(ENC3_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

Stepper loadingMotor(&motor5Step, &motor5Enable);

ProbeSequence probeSequence(&drillMotor, &verticalMotor, &loadingMotor, &verticalEncoder);

// main() runs in its own thread in the OS
int main() {
  loadingMotor.idleCurrent(false);

  while (true) {
    ThisThread::sleep_for(1s);
  }
}
