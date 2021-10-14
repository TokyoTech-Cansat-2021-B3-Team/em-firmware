#include "PinAssignment.h"
#include "mbed.h"

#include "lsm9ds1.h"
#include <cstring>

#define PRINT_BUFFER_SIZE 128

BufferedSerial serial(UART_TX, UART_RX); //
I2C i2c(I2C_SDA, I2C_SCL);               //

char printBuffer[PRINT_BUFFER_SIZE];

#include "PinAssignment.h"

#include "QEI.h"

#include "MotorSpeed.h"
#include "WheelControl.h"
#include "WheelMotor.h"
#include "WheelPID.h"
#include "ekflocalization.h"
#include "fusion-odometry.h"
#include "localization.h"

#define PRINT_BUFFER_SIZE 256

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
MotorSpeed rightMotorSpeed(&rightEncoder, 249.8);

WheelPID leftPID;
WheelPID rightPID;

WheelControl leftControl(&leftWheelMotor, &leftPID, &leftMotorSpeed);
WheelControl rightControl(&rightWheelMotor, &rightPID, &rightMotorSpeed);

FusionOdometry ekf;

EKFLocalization localization(&leftMotorSpeed, &rightMotorSpeed, &imu, &ekf, 180.0e-3, 52.0e-3);

Thread speedThread(osPriorityAboveNormal, 1024, nullptr, nullptr);
Thread printThread(osPriorityAboveNormal, 1024, nullptr, nullptr);

void printThreadLoop() {
  while (true) {
    double tmp = imu.gyrX() * 3.141592653589793 / 180.0;
    snprintf(printBuffer, PRINT_BUFFER_SIZE,
             "Lcsp:%f, Rcsp:%f, w_wh:%f, w_gy:%f, t_kf:%f, w_kf:%f, x_kf:%f, y_kf:%f, v_kf:%f, slip:%f, beta:%f\r\n",
             leftMotorSpeed.currentSpeedRPM(), rightMotorSpeed.currentSpeedRPM(),
             localization.getAngularVelocityFromWheelOdometry(), tmp, localization.theta(), localization.omega(),
             localization.x(), localization.y(), localization.v(),localization.slip(), localization.beta());
    serial.write(printBuffer, strlen(printBuffer));
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
  leftControl.setDirection(FOWARD);
  rightControl.setDirection(FOWARD);
  leftControl.setTargetSpeed(20);
  rightControl.setTargetSpeed(20);
  imu.start();
  localization.start();
  if (imu.getStatus() == LSM9DS1_STATUS_SUCCESS_TO_CONNECT) {
    snprintf(printBuffer, PRINT_BUFFER_SIZE, "Succeeded connecting LSM9DS1.\r\n");
    serial.write(printBuffer, strlen(printBuffer));
  } else {
    snprintf(printBuffer, PRINT_BUFFER_SIZE, "Failed to connect LSM9DS1.\r\n");
    serial.write(printBuffer, strlen(printBuffer));
  }
  while (true) {
    ThisThread::sleep_for(100ms);
  }
}
