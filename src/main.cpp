#include "mbed.h"
#include "PinAssignment.h"

#include <cstring>
#include <exception>

#define PRINT_BUFFER_SIZE 128

BufferedSerial serial(UART_TX, UART_RX);//
I2C i2c(I2C_SDA, I2C_SCL);//

char printBuffer[PRINT_BUFFER_SIZE];


#include "PinAssignment.h"

#include "QEI.h"
#include "lsm9ds1.h"
#include "MU-2.h"
#include "BME280.h"
#include "PA1010D.h"

#include "Fusing.h"

#include "WheelMotor.h"
#include "WheelPID.h"
#include "WheelControl.h"
#include "MotorSpeed.h"
#include "DCMotor.h"
#include "DrillMotor.h"
#include "Stepper.h"

#include "RunningSequence.h"

#define PRINT_BUFFER_SIZE 128

enum ExperimentMode{
    RunningAllSequence,
    RunningPoleToPole,
    RunningNoControle,
};

ExperimentMode flag = RunningNoControle;
const double cruiseSpeed = 40.0;

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

WheelControl leftControl(&leftWheelMotor,&leftPID,&leftMotorSpeed);
WheelControl rightControl(&rightWheelMotor,&rightPID,&rightMotorSpeed);

DigitalIn SafetyPin(FUSE_GATE);

BufferedSerial bufferedSerial(UART_TX, UART_RX, MU2_SERIAL_BAUDRATE);

// main() runs in its own thread in the OS
int main() {
    int i = 0;
    while(true){
        if(SafetyPin==0){
            if(flag!=RunningNoControle)navi.stop();
            leftControl.stop();
            rightControl.stop();

            motor1In1 = 0.0;
            motor1In2 = 0.0;
            motor2In1 = 0.0;
            motor2In2 = 0.0;

            speedThread.terminate();
            printThread.terminate();
            snprintf(printBuffer, PRINT_BUFFER_SIZE, "STOP PROGRAM\r\n");
            serial.write(printBuffer,strlen(printBuffer));
            leftControl.setTargetSpeed(0);
            rightControl.setTargetSpeed(0);
            ThisThread::sleep_for(500ms);


            exit(1);
        }
        if(i < 51){
            snprintf(printBuffer, PRINT_BUFFER_SIZE, "Waiting . . .\r\n");
            serial.write(printBuffer,strlen(printBuffer));
            ThisThread::sleep_for(100ms);
        }else if(i == 51){
            navi.setCruiseSpeed(cruiseSpeed);
            SafetyPin.mode(PullDown);
            printThread.start(printThreadLoop);
            speedThread.start(speedThreadLoop);
        }
        i++;
        ThisThread::sleep_for(10ms);
    }
    
    loadingMotor.idleCurrent(false);

  encoderThread.start(encoderThreadLoop);

  stepperThread.start(stepperThreadLoop);

  while (true) {
    // 上昇
    for (int i = 0; i < 1000; i++) {
      //   printf("%d\n", i);

      leftWheelMotor.forward(i * 0.001);
      rightWheelMotor.forward(i * 0.001);

      verticalMotor.forward(i * 0.001);
      drillMotor.forward(i * 0.001);

      ThisThread::sleep_for(1ms);
    }

    for (int i = 1000; i >= 0; i--) {
      //   printf("%d\n", i);

      leftWheelMotor.forward(i * 0.001);
      rightWheelMotor.forward(i * 0.001);

      verticalMotor.forward(i * 0.001);
      drillMotor.forward(i * 0.001);

      ThisThread::sleep_for(1ms);
    }

    // 下降
    for (int i = 0; i < 1000; i++) {
      //   printf("%d\n", i);

      leftWheelMotor.reverse(i * 0.001);
      rightWheelMotor.reverse(i * 0.001);

      verticalMotor.reverse(i * 0.001);

      ThisThread::sleep_for(1ms);
    }

    for (int i = 1000; i >= 0; i--) {
      //   printf("%d\n", i);

      leftWheelMotor.reverse(i * 0.001);
      rightWheelMotor.reverse(i * 0.001);

      verticalMotor.reverse(i * 0.001);

      ThisThread::sleep_for(1ms);
    }
  }
  mu2.init();

  while (true) {
    const char *str = "Hello MU2";
    mu2.transmit(str, strlen(str));

    ThisThread::sleep_for(5s);
  }

  i2c.frequency(400000);

  bme280.start();

  while (true) {
    printf("press: %lf Pa\n", bme280.getPressure());
    printf("temp: %lf DegC\n", bme280.getTemprature());
    printf("hum: %lf %%RH\n", bme280.getHumidity());

    ThisThread::sleep_for(100ms);
  }
  led = 0;

  ThisThread::sleep_for(10s);

  led = 1;

  fusing.heat(10s);

  led = 0;

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
