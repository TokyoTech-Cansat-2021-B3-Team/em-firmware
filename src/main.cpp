#include "mbed.h"

#include "PinAssignment.h"

#include "BME280.h"
#include "Variometer.h"

I2C i2c(I2C_SDA, I2C_SCL);

BME280 bme280(&i2c);

Variometer variometer(&bme280);

// main() runs in its own thread in the OS
int main() {
  i2c.frequency(400000);

  variometer.start();

  while (true) {
    // printf("$%lf;\n", bme280.getPressure() / 100);
    // printf("temp: %lf DegC\n", bme280.getTemprature());
    // printf("hum: %lf %%RH\n", bme280.getHumidity());
    // printf("alt: %lf m\n", variometer.getAltitude());

    printf("$%lf;\n", variometer.getVerticalSpeed());

    ThisThread::sleep_for(50ms);
  }
}
