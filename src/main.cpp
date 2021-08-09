#include "mbed.h"

#include "PinAssignment.h"

#include "MU2.h"

BufferedSerial bufferedSerial(UART_TX, UART_RX, MU2_SERIAL_BAUDRATE);

MU2 mu2(&bufferedSerial);

// main() runs in its own thread in the OS
int main() {
  mu2.init();

  while (true) {
    const char *str = "Hello MU2";
    mu2.transmit(str, strlen(str));

    ThisThread::sleep_for(5s);
  }
}
