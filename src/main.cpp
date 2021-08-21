#include "mbed.h"

#include "PinAssignment.h"

#include "BME280.h"
#include "LandingSequence.h"
#include "Variometer.h"

I2C i2c(I2C_SDA, I2C_SCL);

BME280 bme280(&i2c);

Variometer variometer(&bme280);

LandingSequence landingSequence(&variometer);

// main() runs in its own thread in the OS
int main() {
  i2c.frequency(400000);

  landingSequence.start();

  while (true) {
    ThisThread::sleep_for(1s);
  }
}
