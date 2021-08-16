#include "mbed.h"

#include "PinAssignment.h"

#include "DrillMotor.h"

PwmOut motor4(M4_IN1);

DrillMotor drillMotor(&motor4);

// main() runs in its own thread in the OS
int main() {
  while (true) {
    drillMotor.forward(1.0);

    ThisThread::sleep_for(2s);

    drillMotor.forward(0.5);

    ThisThread::sleep_for(2s);

    drillMotor.stop();

    ThisThread::sleep_for(2s);
  }
}
