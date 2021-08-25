#include "mbed.h"
#include "LittleFileSystem2.h"
#include "SDBlockDevice.h"

#include "PinAssignment.h"

#include <cstring>

#define PRINT_BUFFER_SIZE 128

BufferedSerial bufferedSerial(UART_TX, UART_RX);//
I2C i2c(I2C_SDA, I2C_SCL);//

char printBuffer[PRINT_BUFFER_SIZE];


#include "PinAssignment.h"

#include "QEI.h"
#include "lsm9ds1.h"
#include "MU2.h"

#include "Console.h"
#include "Logger.h"
#include "fusion-odometry.h"
#include "WheelMotor.h"
#include "WheelPID.h"
#include "WheelControl.h"
#include "MotorSpeed.h"
#include "localization.h"
#include "navigation.h"

#include "RunningSequence.h"

#define PRINT_BUFFER_SIZE 128

PwmOut motor1In1(M1_IN1);
PwmOut motor1In2(M1_IN2);

PwmOut motor2In1(M2_IN1);
PwmOut motor2In2(M2_IN2);

SDBlockDevice sdBlockDevice(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL, 25000000);
LittleFileSystem2 littleFileSystem2(nullptr);

WheelMotor leftWheelMotor(&motor1In1, &motor1In2);
WheelMotor rightWheelMotor(&motor2In1, &motor2In2);

QEI leftEncoder(ENC1_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);
QEI rightEncoder(ENC2_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

LSM9DS1 imu(&i2c);
MU2 mu2(&bufferedSerial);

MotorSpeed leftMotorSpeed(&leftEncoder, 1000.0);
MotorSpeed rightMotorSpeed(&rightEncoder, 249.8);

WheelPID leftPID;
WheelPID rightPID;

Logger logger(&sdBlockDevice, &littleFileSystem2);
Console console(&mu2, &logger);

WheelControl leftControl(&leftWheelMotor,&leftPID,&leftMotorSpeed);
WheelControl rightControl(&rightWheelMotor,&rightPID,&rightMotorSpeed);

FusionOdometry ekf(KALMANFILTER_PERIOD);

Localization localization(&leftMotorSpeed, &rightMotorSpeed, &imu, &ekf, 180.0e-3, 68.0e-3);

Navigation navi(&localization, &leftControl, &rightControl);

RunningSequence runningSequence(&navi, &localization, &imu, &leftMotorSpeed, &rightMotorSpeed, &leftControl, &rightControl, &console);

Thread speedThread(osPriorityAboveNormal, 1024, nullptr, nullptr);
Thread printThread(osPriorityAboveNormal, 1024, nullptr, nullptr);

void printThreadLoop(){
    while(true){
        //snprintf(printBuffer, PRINT_BUFFER_SIZE, "$%d %f %f %f %f %f %f %f;\r\n",runningSequence.state(), navi.leftTargetSpeed(), navi.rightTargetSpeed(),leftMotorSpeed.currentSpeedRPM(), rightMotorSpeed.currentSpeedRPM() ,localization.theta(), localization.x(), localization.y());
        //serial.write(printBuffer,strlen(printBuffer));
        ThisThread::sleep_for(20ms);
    }
}


// main() runs in its own thread in the OS
int main() {
  runningSequence.start(FIRST);
  if(imu.getStatus()==LSM9DS1_STATUS_SUCCESS_TO_CONNECT){
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "Succeeded connecting LSM9DS1.\r\n");
      //bufferedSerial.write(printBuffer,strlen(printBuffer));
  }else{
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "Failed to connect LSM9DS1.\r\n");
      //bufferedSerial.write(printBuffer,strlen(printBuffer));
  }
  printThread.start(printThreadLoop);
  int i = 0;
  while(true){
      if(runningSequence.state() == ARRIVED_SECOND_POLE){
          runningSequence.stop();
          ThisThread::sleep_for(5s);
          runningSequence.start(SECOND);
      }
      if(runningSequence.state() == ARRIVED_THIRD_POLE){
          runningSequence.stop();
          ThisThread::sleep_for(5s);
          runningSequence.start(THIRD);
      }
      if(runningSequence.state() == ARRIVED_FOURTH_POLE){
          runningSequence.stop();
          snprintf(printBuffer, PRINT_BUFFER_SIZE, "SUCCESS\r\n");
          //bufferedSerial.write(printBuffer,strlen(printBuffer));
          while(true){
            ThisThread::sleep_for(100ms);
          }
      }
      ThisThread::sleep_for(100ms);
  }
}
