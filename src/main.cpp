#include "mbed.h"

#include "LittleFileSystem2.h"
#include "SDBlockDevice.h"

#include "PinAssignment.h"

#include "Logger.h"
#include "PA1010D.h"

#include "TDBStore.h"

I2C i2c(I2C_SDA, I2C_SCL);

PA1010D pa1010d(&i2c);

#define BLOCK_SIZE 512

SDBlockDevice sdBlockDevice(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL, 25000000);
SlicingBlockDevice kvBlockDevice(&sdBlockDevice, 0 * BLOCK_SIZE, 0x8000 * BLOCK_SIZE);
SlicingBlockDevice fsBlockDevice(&sdBlockDevice, 0x8000 * BLOCK_SIZE, 0x1CF8000 * BLOCK_SIZE);

LittleFileSystem2 littleFileSystem2(nullptr);

TDBStore tdbStore(&kvBlockDevice);

Logger logger(&sdBlockDevice, &littleFileSystem2, &tdbStore);

// main() runs in its own thread in the OS
int main() {
  logger.init();
  pa1010d.start();

  logger.writeStore(LOGGER_KEY_COMPLETE_LANDING, false);

  bool completeLanding = false;
  logger.readStore(LOGGER_KEY_COMPLETE_LANDING, &completeLanding);

  printf("%d\n", completeLanding);

  logger.writeStore(LOGGER_KEY_COMPLETE_LANDING, true);

  completeLanding = false;
  logger.readStore(LOGGER_KEY_COMPLETE_LANDING, &completeLanding);

  printf("%d\n", completeLanding);

  while (true) {
    PA1010D::GGAPacket packet;

    pa1010d.getGGA(&packet);

    Logger::GPSLogData logData = {
        packet.utc,                 //
        packet.latitude,            //
        packet.nsIndicator,         //
        packet.longitude,           //
        packet.ewIndicator,         //
        packet.positionFixIndicator //
    };

    logger.gpsLog(&logData);

    logger.lprintf("main", "LogMessage\n");

    ThisThread::sleep_for(1s);
  }

  // logger.deinit();
}
