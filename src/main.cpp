#include "mbed.h"

#include "PinAssignment.h"

#include "Fusing.h"

PwmOut fuseGate(FUSE_GATE);

Fusing fusing(&fuseGate);

DigitalOut led(LED1);

// main() runs in its own thread in the OS
int main() {
  led = 0;

  ThisThread::sleep_for(10s);

  led = 1;

  fusing.heat(10s);

  led = 0;
}
