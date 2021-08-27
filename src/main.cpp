#include "mbed.h"

#include "PinAssignment.h"

#include "BME280.h"
#include "MU2.h"
#include "PA1010D.h"
#include "QEI.h"
#include "lsm9ds1.h"

#include "Fusing.h"

#include "DCMotor.h"
#include "DrillMotor.h"
#include "MotorSpeed.h"
#include "Stepper.h"
#include "WheelControl.h"
#include "WheelMotor.h"
#include "WheelPID.h"

#define PRINT_BUFFER_SIZE 256

enum ExperimentMode { CHECK_ALL };

char printBuffer[PRINT_BUFFER_SIZE];

ExperimentMode flag = CHECK_ALL;

BufferedSerial bufferedSerial(UART_TX, UART_RX, MU2_SERIAL_BAUDRATE);

PwmOut motor1In1(M1_IN1);
PwmOut motor1In2(M1_IN2);

PwmOut motor2In1(M2_IN1);
PwmOut motor2In2(M2_IN2);

PwmOut motor3In1(M3_IN1);
PwmOut motor3In2(M3_IN2);

PwmOut motor4In1(M4_IN1);

PwmOut fuseGate(FUSE_GATE);

DigitalOut motor5Enable(M5_ENABLE);
DigitalOut motor5Step(M5_STEP);

WheelMotor leftWheelMotor(&motor1In1, &motor1In2);
WheelMotor rightWheelMotor(&motor2In1, &motor2In2);

DCMotor verticalMotor(&motor3In1, &motor3In2);
DrillMotor drillMotor(&motor4In1);
QEI verticalEncoder(ENC3_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

Stepper loadingMotor(&motor5Step, &motor5Enable);
QEI leftEncoder(ENC1_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);
QEI rightEncoder(ENC2_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

I2C i2c(I2C_SDA, I2C_SCL);

LSM9DS1 imu(&i2c);
MU2 mu2(&bufferedSerial);
BME280 bme280(&i2c);

PA1010D pa1010d(&i2c);

Fusing fusing(&fuseGate);
DigitalOut led(LED1);

MotorSpeed leftMotorSpeed(&leftEncoder, 298.0);
MotorSpeed rightMotorSpeed(&rightEncoder, 298.0);

WheelPID leftPID;
WheelPID rightPID;

WheelControl leftControl(&leftWheelMotor, &leftPID, &leftMotorSpeed);
WheelControl rightControl(&rightWheelMotor, &rightPID, &rightMotorSpeed);

// main() runs in its own thread in the OS
int main() {
  if (flag == CHECK_ALL) {
    // while (true) {
    //   led = !led;
    //   ThisThread::sleep_for(1s);
    // }

    // // Check MU-2
    // snprintf(printBuffer, PRINT_BUFFER_SIZE, "Check MU-2\r\n");
    // mu2.transmit(printBuffer, strlen(printBuffer));
    // mu2.init();
    // while (true) {
    //   const char *str = "MU2 Downlink";
    //   mu2.transmit(str, strlen(str));

    //   ThisThread::sleep_for(1s);
    // }

    // init GPIO
    mu2.init();

    snprintf(printBuffer, PRINT_BUFFER_SIZE, "Initializing GPIO and Setting I2C");
    mu2.transmit(printBuffer, strlen(printBuffer));

    loadingMotor.idleCurrent(false);
    i2c.frequency(400000);

    // // Check Wheel Motor
    // snprintf(printBuffer, PRINT_BUFFER_SIZE, "Check Wheel Motor\r\n");
    // bufferedSerial.write(printBuffer, strlen(printBuffer));
    // snprintf(printBuffer, PRINT_BUFFER_SIZE, "Target Speed is 15RPM\r\n");
    // bufferedSerial.write(printBuffer, strlen(printBuffer));
    // leftMotorSpeed.start();
    // rightMotorSpeed.start();
    // leftControl.setTargetSpeed(15);
    // rightControl.setTargetSpeed(15);
    // // 5s間実行
    // for (int i = 0; i < 50; i++) {
    //   snprintf(printBuffer, PRINT_BUFFER_SIZE, "L : %f \t R : %f [RPM]\r\n", leftMotorSpeed.currentSpeedRPM(),
    //            rightMotorSpeed.currentSpeedRPM());
    //   bufferedSerial.write(printBuffer, strlen(printBuffer));
    //   ThisThread::sleep_for(100ms);
    // }
    // snprintf(printBuffer, PRINT_BUFFER_SIZE, "Left and Right Motor OFF\r\n");
    // bufferedSerial.write(printBuffer, strlen(printBuffer));
    // leftControl.setTargetSpeed(0);
    // rightControl.setTargetSpeed(0);
    // ThisThread::sleep_for(1s);

    // leftWheelMotor.reverse(1.0);
    // rightWheelMotor.reverse(1.0);

    // for (int i = 0; i < 10; i++) {
    //   snprintf(printBuffer, PRINT_BUFFER_SIZE, "left: %d, right: %d\r\n", leftEncoder.getPulses(),
    //            rightEncoder.getPulses());
    //   mu2.transmit(printBuffer, strlen(printBuffer));
    //   ThisThread::sleep_for(1s);
    // }

    // leftWheelMotor.stop();
    // rightWheelMotor.stop();

    // drillMotor.forward(1.0);
    // ThisThread::sleep_for(5s);
    // drillMotor.stop();

    // printf("%d\n", verticalEncoder.getPulses());
    // verticalMotor.reverse(0.5);
    // ThisThread::sleep_for(5s);
    // verticalMotor.forward(0.5);
    // ThisThread::sleep_for(5s);
    // verticalMotor.stop();

    // while (true) {
    //   ThisThread::sleep_for(1s);
    // }

    // while (true) {
    //   loadingMotor.idleCurrent(true);
    //   loadingMotor.rotate(10, 0.01);
    //   loadingMotor.idleCurrent(false);
    //   //   ThisThread::sleep_for(1s);
    // }

    // Check LSM9DS1
    snprintf(printBuffer, PRINT_BUFFER_SIZE, "Check LSM9DS1");
    mu2.transmit(printBuffer, strlen(printBuffer));
    imu.start();
    if (imu.getStatus() == LSM9DS1_STATUS_SUCCESS_TO_CONNECT) {
      imu.start();
      // 2s間みる
      for (int i = 0; i < 10; i++) {
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "$%f %f %f %f %f %f %f %f %f;", imu.accX(), imu.accY(), imu.accZ(),
                 imu.gyrX(), imu.gyrY(), imu.gyrZ(), imu.magX(), imu.magY(), imu.magZ());
        mu2.transmit(printBuffer, strlen(printBuffer));
        ThisThread::sleep_for(1s);
      }
      imu.stop();
    }

    // Check bme280
    snprintf(printBuffer, PRINT_BUFFER_SIZE, "Check BME280");
    mu2.transmit(printBuffer, strlen(printBuffer));
    bme280.start();
    for (int i = 0; i < 10; i++) {

      snprintf(printBuffer, PRINT_BUFFER_SIZE, "press: %lf Pa, temp: %lf DegC", bme280.getPressure(),
               bme280.getTemprature());
      mu2.transmit(printBuffer, strlen(printBuffer));

      ThisThread::sleep_for(1s);
    }

    // Check PA1010D
    snprintf(printBuffer, PRINT_BUFFER_SIZE, "Check PA1010A");
    mu2.transmit(printBuffer, strlen(printBuffer));

    pa1010d.start();

    PA1010D::GGAPacket gga;
    for (int i = 0; i < 5; i++) {

      pa1010d.getGGA(&gga);

      snprintf(printBuffer, PRINT_BUFFER_SIZE, "utc: %lf, lat: %lf, ns: %c, lng: %lf, ew: %c, fix: %u", gga.utc,
               gga.latitude, gga.nsIndicator ? gga.nsIndicator : ' ', gga.longitude,
               gga.ewIndicator ? gga.ewIndicator : ' ', gga.positionFixIndicator);
      mu2.transmit(printBuffer, strlen(printBuffer));

      ThisThread::sleep_for(1s);
    }

    // // Check Fuse
    // snprintf(printBuffer, PRINT_BUFFER_SIZE, "Check Fuse in 5s");
    // mu2.transmit(printBuffer, strlen(printBuffer));

    // snprintf(printBuffer, PRINT_BUFFER_SIZE, "Heating will start in 10s. CAUTION!!!!!!");
    // mu2.transmit(printBuffer, strlen(printBuffer));

    // led = 0;
    // ThisThread::sleep_for(10s);

    // snprintf(printBuffer, PRINT_BUFFER_SIZE, "Heating start");
    // mu2.transmit(printBuffer, strlen(printBuffer));

    // led = 1;
    // fusing.heat(10s);

    // snprintf(printBuffer, PRINT_BUFFER_SIZE, "Cooling Start");
    // mu2.transmit(printBuffer, strlen(printBuffer));

    // led = 0;
    // ThisThread::sleep_for(2s);

    while (true) {
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "All Checking is DONE");
      mu2.transmit(printBuffer, strlen(printBuffer));
      ThisThread::sleep_for(1s);
    }
  }
}
