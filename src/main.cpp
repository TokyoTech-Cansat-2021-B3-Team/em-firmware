#include "mbed.h"

#include "PinAssignment.h"

PwmOut motor1In1(M1_IN1);
PwmOut motor2In1(M2_IN1);

// main() runs in its own thread in the OS
int main() {
  while (true) {
    printf("Hello\n");

    motor1In1 = 0.01;
    motor2In1 = 0.01;

    ThisThread::sleep_for(100ms);
  }
}
