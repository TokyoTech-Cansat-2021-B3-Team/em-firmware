#include "mbed.h"

// includes
#include "PinAssignment.h"

// embedded
#include "LittleFileSystem2.h"
#include "SDBlockDevice.h"

// drivers
#include "MU2.h"
#include "PA1010D.h"

// middlewares
#include "Console.h"
#include "Logger.h"

// sequences
#include "GPSDownlink.h"

// defines
#define SPI_FREQUENCY 25000000
#define I2C_FREQUENCY 400000

// objects

// embedded
I2C i2c(I2C_SDA, I2C_SCL);
BufferedSerial bufferedSerial(UART_TX, UART_RX, MU2_SERIAL_BAUDRATE);
SDBlockDevice sdBlockDevice(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL, SPI_FREQUENCY);
LittleFileSystem2 littleFileSystem2(nullptr);

// drivers
MU2 mu2(&bufferedSerial);
PA1010D pa1010d(&i2c);

// middlewares
Logger logger(&sdBlockDevice, &littleFileSystem2);
Console console(&mu2, &logger);

// sequences
GPSDownlink gpsDownlink(&pa1010d, &console, &logger);

// main() runs in its own thread in the OS
int main() {
  // I2C速度変更
  i2c.frequency(I2C_FREQUENCY);

  gpsDownlink.start();

  while (true) {
    ThisThread::sleep_for(1s);
  }
}
