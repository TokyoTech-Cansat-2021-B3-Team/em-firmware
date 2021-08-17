#include "mbed.h"

#include "PinAssignment.h"

PwmOut motor1In1(M1_IN1);
PwmOut motor2In1(M2_IN1);

// main() runs in its own thread in the OS
int main() {

  while (true) {
    for (int i = 0; i < 1000; i++) {
      printf("%d\n", i);

      motor1In1 = i * 0.001;
      motor2In1 = i * 0.001;

      ThisThread::sleep_for(10ms);
    }

    for (int i = 1000; i >= 0; i--) {
      printf("%d\n", i);

      motor1In1 = i * 0.001;
      motor2In1 = i * 0.001;

      ThisThread::sleep_for(10ms);
    }

    ThisThread::sleep_for(100ms);
  }
}
