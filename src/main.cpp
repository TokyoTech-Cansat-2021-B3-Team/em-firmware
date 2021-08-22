#include "mbed.h"

#include "LittleFileSystem2.h"
#include "SDBlockDevice.h"

#include "PinAssignment.h"

#include "Logger.h"
#include "PA1010D.h"

I2C i2c(I2C_SDA, I2C_SCL);

PA1010D pa1010d(&i2c);

SDBlockDevice sdBlockDevice(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL, 25000000);

LittleFileSystem2 littleFileSystem2(nullptr);

Logger logger(&sdBlockDevice, &littleFileSystem2);

// main() runs in its own thread in the OS
int main() {
  logger.init();
  pa1010d.start();

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
