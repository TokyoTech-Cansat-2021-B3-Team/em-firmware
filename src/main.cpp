#include "mbed.h"

#include "Mu2.h"

BufferedSerial bufferedSerial(PA_2, NC, MU2_SERIAL_BAUDRATE);

Mu2 mu2(&bufferedSerial);

// main() runs in its own thread in the OS
int main() {
  mu2.init();

  while (true) {
    const char *str = "Hello Mu2";
    mu2.transmit(str, strlen(str));

    ThisThread::sleep_for(5s);
  }
}