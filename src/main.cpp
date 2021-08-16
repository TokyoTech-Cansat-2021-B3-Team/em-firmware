#include "mbed.h"

#include "PinAssignment.h"

#include "PA1010D.h"

I2C i2c(I2C_SDA, I2C_SCL);

PA1010D pa1010d(&i2c);

// main() runs in its own thread in the OS
int main() {
  i2c.frequency(400000);

  pa1010d.start();

  PA1010D::RMCPacket rmc;

  while (true) {
    pa1010d.getRMC(&rmc);

    printf("utc: %lf\n", rmc.utc);
    printf("status: %c\n", rmc.status);
    printf("latitude: %lf\n", rmc.latitude);
    printf("nsIndicator: %c\n", rmc.nsIndicator);
    printf("longitude: %lf\n", rmc.longitude);
    printf("ewIndicator: %c\n", rmc.ewIndicator);
    printf("speedOverGround: %f\n", rmc.speedOverGround);
    printf("courseOverGround: %f\n", rmc.courseOverGround);
    printf("date: %u\n", rmc.date);
    printf("magneticVariation: %f\n", rmc.magneticVariation);
    printf("variationDirection: %c\n", rmc.variationDirection);
    printf("mode: %c\n", rmc.mode);

    ThisThread::sleep_for(1s);
  }
}
