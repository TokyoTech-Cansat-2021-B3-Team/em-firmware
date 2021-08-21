#include "mbed.h"

#include "PinAssignment.h"

#include "BME280.h"
#include "LandingSequence.h"
#include "Variometer.h"
#include <cstdio>

I2C i2c(I2C_SDA, I2C_SCL);

BME280 bme280(&i2c);

Variometer variometer(&bme280);

LandingSequence landingSequence(&variometer);

void startSync() {
  landingSequence.start();

  printf("start\n");

  while (landingSequence.state() != LandingSequence::Complete) {
    ThisThread::sleep_for(100ms);
  }

  landingSequence.stop();

  printf("stop\n");
}

// main() runs in its own thread in the OS
int main() {
  i2c.frequency(400000);

  startSync();
}
