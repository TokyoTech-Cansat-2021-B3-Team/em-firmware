#include "mbed.h"

#include "PinAssignment.h"

#include "BME280.h"

I2C i2c(I2C_SDA, I2C_SCL);

BME280 bme280(&i2c);

// main() runs in its own thread in the OS
int main() {
  i2c.frequency(400000);

  bme280.start();

  while (true) {
    printf("press: %lf Pa\n", bme280.getPressure());
    printf("temp: %lf DegC\n", bme280.getTemprature());
    printf("hum: %lf %%RH\n", bme280.getHumidity());

    ThisThread::sleep_for(100ms);
  }
}
