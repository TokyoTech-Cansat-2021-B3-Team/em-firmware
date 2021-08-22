#include "mbed.h"

#include "LittleFileSystem2.h"
#include "SDBlockDevice.h"

#include "PinAssignment.h"

#include "MU2.h"
#include "PA1010D.h"

#include "Console.h"
#include "Logger.h"

I2C i2c(I2C_SDA, I2C_SCL);
PA1010D pa1010d(&i2c);

SDBlockDevice sdBlockDevice(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL, 25000000);
LittleFileSystem2 littleFileSystem2(nullptr);

BufferedSerial bufferedSerial(UART_TX, UART_RX, MU2_SERIAL_BAUDRATE);
MU2 mu2(&bufferedSerial);

Logger logger(&sdBlockDevice, &littleFileSystem2);
Console console(&mu2);

// main() runs in its own thread in the OS
int main() {
  console.init();

  while (true) {
    console.lprintf("main", "log and downlink message\n");

    ThisThread::sleep_for(1s);
  }
}
