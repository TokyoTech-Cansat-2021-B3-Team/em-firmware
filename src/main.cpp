#include "mbed.h"

#include "LittleFileSystem.h"
#include "Logger.h"
#include "SDBlockDevice.h"

#include "PinAssignment.h"

SDBlockDevice sdBlockDevice(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL, 25000000);

LittleFileSystem littleFileSystem(nullptr);

Logger logger(&sdBlockDevice, &littleFileSystem);

// main() runs in its own thread in the OS
int main() {
  logger.init();

  Timer t;
  t.start();
  logger.lprintf("Hello!\n");
  t.stop();

  // logger.deinit();

  printf("timer: %llu\n", t.elapsed_time().count());

  while (true) {
    ThisThread::sleep_for(1s);
  }
}
