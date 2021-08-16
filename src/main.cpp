#include "mbed.h"

#include "PinAssignment.h"

#include "PA1010D.h"

I2C i2c(I2C_SDA, I2C_SCL);

PA1010D pa1010d(&i2c);

// main() runs in its own thread in the OS
int main() {
  ThisThread::sleep_for(2s);

  i2c.frequency(400000);

  pa1010d.start();

  //   PA1010D::RMCPacket rmc;
  PA1010D::GGAPacket gga;

  while (true) {
    // pa1010d.getRMC(&rmc);

    // printf("utc: %lf\n", rmc.utc);
    // printf("status: %c\n", rmc.status);
    // printf("latitude: %lf\n", rmc.latitude);
    // printf("nsIndicator: %c\n", rmc.nsIndicator);
    // printf("longitude: %lf\n", rmc.longitude);
    // printf("ewIndicator: %c\n", rmc.ewIndicator);
    // printf("speedOverGround: %f\n", rmc.speedOverGround);
    // printf("courseOverGround: %f\n", rmc.courseOverGround);
    // printf("date: %u\n", rmc.date);
    // printf("magneticVariation: %f\n", rmc.magneticVariation);
    // printf("variationDirection: %c\n", rmc.variationDirection);
    // printf("mode: %c\n", rmc.mode);

    pa1010d.getGGA(&gga);

    printf("utc: %lf\n", gga.utc);
    printf("latitude: %lf\n", gga.latitude);
    printf("nsIndicator: %c\n", gga.nsIndicator);
    printf("longitude: %lf\n", gga.longitude);
    printf("ewIndicator: %c\n", gga.ewIndicator);
    printf("positionFixIndicator: %u\n", gga.positionFixIndicator);
    printf("satellitesUsed: %u\n", gga.satellitesUsed);
    printf("hdop: %f\n", gga.hdop);
    printf("mslAltitude: %f\n", gga.mslAltitude);
    printf("geoidalSeparation: %f\n", gga.geoidalSeparation);
    printf("ageOfDiffCorr: %f\n", gga.ageOfDiffCorr);
    printf("stationID: %u\n", gga.stationID);

    ThisThread::sleep_for(5s);
  }
}
