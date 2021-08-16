#include "mbed.h"

#include "PinAssignment.h"

PwmOut fuse(FUSE_GATE);

DigitalOut led(LED1);

Timer timer;

// main() runs in its own thread in the OS
int main() {
  fuse = 0.0;
  led = 0;

  for (int i = 0; i < 10; i++) {
    printf("%d\n", 10 - i);
    ThisThread::sleep_for(1s);
  }

  timer.start();

  fuse = 1.0;
  led = 1;

  while (true) {
    printf("%llu\n", chrono::duration_cast<chrono::milliseconds>(timer.elapsed_time()).count());

    ThisThread::sleep_for(1s);
  }
}
