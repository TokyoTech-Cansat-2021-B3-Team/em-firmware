#include "PinAssignment.h"

#include "LittleFileSystem2.h"
#include "SDBlockDevice.h"

#include "mbed.h"

#include "lsm9ds1.h"
#include <cstdio>
#include <cstring>
#include <exception>

#include "QEI.h"

#include "MotorSpeed.h"
#include "WheelControl.h"
#include "WheelMotor.h"
#include "WheelPID.h"
#include "ekflocalization.h"
#include "simplelocalization.h"
#include "fusion-odometry.h"
#include "localization.h"
#include "navigation.h"

#include "RunningSequence.h"

#define PRINT_BUFFER_SIZE 384

BufferedSerial bufferedSerial(UART_TX, UART_RX); //
I2C i2c(I2C_SDA, I2C_SCL);                       //

char printBuffer[PRINT_BUFFER_SIZE];

enum ExperimentMode { RunningAllSequence, RunningPoleToPole, RunningNoControle };

ExperimentMode flag = RunningAllSequence;
const double cruiseSpeed = 20.0;
#define PRINT_BUFFER_SIZE 256

PwmOut motor1In1(M1_IN1);
PwmOut motor1In2(M1_IN2);

PwmOut motor2In1(M2_IN1);
PwmOut motor2In2(M2_IN2);

DigitalOut motor5Enable(M5_ENABLE);

DigitalIn saftyPin(FUSE_GATE);

SDBlockDevice sdBlockDevice(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL, 25000000);
LittleFileSystem2 littleFileSystem2(nullptr);

WheelMotor leftWheelMotor(&motor1In1, &motor1In2);
WheelMotor rightWheelMotor(&motor2In2, &motor2In1);

QEI leftEncoder(ENC1_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);
QEI rightEncoder(ENC2_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

LSM9DS1 imu(&i2c);
MU2 mu2(&bufferedSerial);

MotorSpeed leftMotorSpeed(&leftEncoder, 986.41);
MotorSpeed rightMotorSpeed(&rightEncoder, 986.41);

WheelPID leftPID;
WheelPID rightPID;

Logger logger(&sdBlockDevice, &littleFileSystem2);
Console console(&mu2, &logger);

WheelControl leftControl(&leftWheelMotor, &leftPID, &leftMotorSpeed);
WheelControl rightControl(&rightWheelMotor, &rightPID, &rightMotorSpeed);

FusionOdometry ekf;

SimpleLocalization simpleLocalization(&leftMotorSpeed, &rightMotorSpeed, 180.0e-3, 68.0e-3);
EKFLocalization localization(&leftMotorSpeed, &rightMotorSpeed, &imu, &ekf, 180.0e-3, 52.0e-3);

Navigation navi(&localization, &leftControl, &rightControl);

RunningSequence runningSequence(&navi, &localization, &imu, &leftMotorSpeed, &rightMotorSpeed, &leftControl,
                                &rightControl, &console, &logger);

Thread speedThread(osPriorityAboveNormal, 1024, nullptr, nullptr);
Thread printThread(osPriorityAboveNormal, 1024, nullptr, nullptr);

void printThreadLoop() {
  snprintf(printBuffer, PRINT_BUFFER_SIZE, "stts Ltsp Rtsp Lcsp Rcsp w_wh w_gy t_kf w_kf x_kf y_kf v_kf t_sm x_sm y_sm'\r\n");
  //snprintf(printBuffer, PRINT_BUFFER_SIZE, "stat Ltsp Rtsp Lcsp Rcsp t_kf x_kf y_kf\r\n");
  bufferedSerial.write(printBuffer, strlen(printBuffer));
  while (true) {
       
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "$%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f;\r\n",
                 runningSequence.state(), navi.leftTargetSpeed(), navi.rightTargetSpeed(),
       leftMotorSpeed.currentSpeedRPM(), rightMotorSpeed.currentSpeedRPM(),
       localization.getAngularVelocityFromWheelOdometry(), -imu.gyrY(), localization.theta(), localization.omega(),
       localization.x(), localization.y(), localization.v(), simpleLocalization.theta(), simpleLocalization.x(),
       simpleLocalization.y());
       
    //snprintf(printBuffer, PRINT_BUFFER_SIZE, "%f %f %f \r\n", localization.x(), localization.y(),sqrt(localization.x()*localization.x()+localization.y()*localization.y()));
    // snprintf(printBuffer, PRINT_BUFFER_SIZE, "$%d %f %f %f %f %f %f %f;\r\n", runningSequence.state(),
    //         navi.leftTargetSpeed(), navi.rightTargetSpeed(), leftMotorSpeed.currentSpeedRPM(),
    //         rightMotorSpeed.currentSpeedRPM(), localization.theta(), localization.x(), localization.y());

             bufferedSerial.write(printBuffer, strlen(printBuffer));
             ThisThread::sleep_for(20ms);
  }
}

void speedThreadLoop() {
  if (flag == RunningAllSequence) {
    runningSequence.start(FIRST);
    simpleLocalization.start();
    if (imu.getStatus() == LSM9DS1_STATUS_SUCCESS_TO_CONNECT) {
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "Succeeded connecting LSM9DS1.\r\n");
      bufferedSerial.write(printBuffer, strlen(printBuffer));
    } else {
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "Failed to connect LSM9DS1.\r\n");
      bufferedSerial.write(printBuffer, strlen(printBuffer));
    }
    int i = 0;
    while (true) {
      if (runningSequence.state() == ARRIVED_SECOND_POLE) {
        runningSequence.stop();
        ThisThread::sleep_for(30s);
        runningSequence.start(SECOND);
      }
      if (runningSequence.state() == ARRIVED_THIRD_POLE) {
        runningSequence.stop();
        ThisThread::sleep_for(30s);
        runningSequence.start(THIRD);
      }
      if (runningSequence.state() == ARRIVED_FOURTH_POLE) {
        runningSequence.stop();
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "SUCCESS\r\n");
        bufferedSerial.write(printBuffer, strlen(printBuffer));
        while (true) {
          ThisThread::sleep_for(100ms);
        }
      }
      ThisThread::sleep_for(100ms);
    }
  } else if (flag == RunningPoleToPole) {
    imu.start();
    leftMotorSpeed.start();
    rightMotorSpeed.start();
    localization.start();
    simpleLocalization.start();
    leftControl.start();
    rightControl.start();
    navi.start();
    navi.setTargetPosition(5.0, 0.0, 0.1);
  } else if (flag == RunningNoControle) {
    imu.start();
    leftMotorSpeed.start();
    rightMotorSpeed.start();
    localization.start();
    simpleLocalization.start();
    leftControl.start();
    rightControl.start();
    leftControl.setTargetSpeed(cruiseSpeed);
    rightControl.setTargetSpeed(cruiseSpeed);
    navi.setTargetPosition(5.0, 0.0, 0.5);
    while (true) {
      if (navi.checkArrivingTarget()) {
        leftControl.setTargetSpeed(0.0);
        rightControl.setTargetSpeed(0.0);
      }
      ThisThread::sleep_for(100ms);
    }
  }
}

// main() runs in its own thread in the OS
int main() {
  int i = 0;
  // logger.init();
  // console.init();
  motor5Enable = 0;
  // leftControl.setDirection(REVERSE);
  // rightControl.setDirection(REVERSE);
  while (true) {
    if (i < 51) {
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "Waiting . . .\r\n");
      bufferedSerial.write(printBuffer, strlen(printBuffer));
      ThisThread::sleep_for(100ms);
    } else if (i == 51) {
      navi.setCruiseSpeed(cruiseSpeed);
      printThread.start(printThreadLoop);
      speedThread.start(speedThreadLoop);
    }
    if (saftyPin != 1) {
      leftControl.setTargetSpeed(0);
      rightControl.setTargetSpeed(0);
      leftWheelMotor.stop();
      rightWheelMotor.stop();
      speedThread.terminate();
      navi.stop();
      runningSequence.stop();
      printThread.terminate();
      printf("stop\r\n");
      while (true) {
        ThisThread::sleep_for(10ms);
      }
    }
    i++;
    ThisThread::sleep_for(10ms);
  }
}
