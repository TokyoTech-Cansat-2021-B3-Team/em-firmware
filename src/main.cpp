#include "mbed.h"

#include "PinAssignment.h"

#include "Console.h"
#include "MU2.h"

BufferedSerial bufferedSerial(UART_TX, UART_RX, MU2_SERIAL_BAUDRATE);

MU2 mu2(&bufferedSerial);

Console console(&mu2);

// main() runs in its own thread in the OS
int main() {
  console.init();

  while (true) {
    console.lprintf("main", "downlink message\n");

    ThisThread::sleep_for(1s);
  }
}
