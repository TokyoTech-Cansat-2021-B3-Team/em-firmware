#include "mbed.h"
#include "PinAssignment.h"

#include "lsm9ds1.h"
#include <cstring>

#define PRINT_BUFFER_SIZE 128

BufferedSerial serial(UART_TX, UART_RX);//
I2C i2c(I2C_SDA, I2C_SCL);//

char printBuffer[PRINT_BUFFER_SIZE];


#include "PinAssignment.h"

#include "QEI.h"

#include "fusion-odometry.h"
#include "WheelMotor.h"
#include "WheelPID.h"
#include "WheelControl.h"
#include "MotorSpeed.h"
#include "Localization.h"
#include "navigation.h"
#include "TorqueControl.h"

#define PRINT_BUFFER_SIZE 128

PwmOut motor1In1(M1_IN1);
PwmOut motor1In2(M1_IN2);

PwmOut motor2In1(M2_IN1);
PwmOut motor2In2(M2_IN2);

WheelMotor leftWheelMotor(&motor1In1, &motor1In2);
WheelMotor rightWheelMotor(&motor2In1, &motor2In2);

QEI leftEncoder(ENC1_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);
QEI rightEncoder(ENC2_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

LSM9DS1 imu(&i2c);

MotorSpeed leftMotorSpeed(&leftEncoder, 1000.0);
MotorSpeed rightMotorSpeed(&rightEncoder, 1000.0);

WheelPID leftPID;
WheelPID rightPID;

WheelControl leftControl(&leftWheelMotor,&leftPID,&leftMotorSpeed);
WheelControl rightControl(&rightWheelMotor,&rightPID,&rightMotorSpeed);

FusionOdometry ekf;

Localization localization(&leftMotorSpeed, &rightMotorSpeed, &imu, &ekf, 180.0e-3, 52.0e-3);

TorqueControl torqueControl(&leftMotorSpeed, &rightMotorSpeed, &leftControl, &rightControl, &leftPID, &rightPID);
Navigation navi(&localization, &leftControl, &rightControl, &torqueControl);

Thread speedThread(osPriorityAboveNormal, 1024, nullptr, nullptr);
Thread printThread(osPriorityAboveNormal, 1024, nullptr, nullptr);

void printThreadLoop(){
    while(true){
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "$%f %f %f %f %f %f %f;\r\n", navi.leftTargetSpeed(), navi.rightTargetSpeed(),leftMotorSpeed.currentSpeedRPM(), rightMotorSpeed.currentSpeedRPM() ,localization.theta(), localization.x(), localization.y());
        serial.write(printBuffer,strlen(printBuffer));
        ThisThread::sleep_for(500ms);
    }
}

// main() runs in its own thread in the OS
int main() {
  printThread.start(printThreadLoop);
  leftMotorSpeed.start();
  rightMotorSpeed.start();
  leftControl.start();
  rightControl.start();
  torqueControl.setGeneralCruiseSpeed(20);
  torqueControl.setSlowCruiseSpeed(5);
  torqueControl.start();
  leftControl.setTargetSpeed(20);
  rightControl.setTargetSpeed(20);
  imu.start();
  localization.start();
  navi.setTargetPosition(10.0,0.0,0.1);
  navi.start();
  if(imu.getStatus()==LSM9DS1_STATUS_SUCCESS_TO_CONNECT){
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "Succeeded connecting LSM9DS1.\r\n");
      serial.write(printBuffer,strlen(printBuffer));
  }else{
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "Failed to connect LSM9DS1.\r\n");
      serial.write(printBuffer,strlen(printBuffer));
  }
  while(true){
      ThisThread::sleep_for(100ms);
  }
}
