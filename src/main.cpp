#include "LittleFileSystem2.h"
#include "SDBlockDevice.h"
#include "mbed.h"

#include "PinAssignment.h"
#include "MissionParameters.h"

#define PRINT_BUFFER_SIZE 512

BufferedSerial bufferedSerial(UART_TX, UART_RX); //
I2C i2c(I2C_SDA, I2C_SCL);                       //

char printBuffer[PRINT_BUFFER_SIZE];

#include "MU2.h"
#include "QEI.h"
#include "lsm9ds1.h"

#include "Console.h"
#include "Logger.h"
#include "MotorSpeed.h"
#include "TorqueControl.h"
#include "WheelControl.h"
#include "WheelMotor.h"
#include "WheelPID.h"
#include "ekflocalization.h"
#include "fusion-odometry.h"
#include "localization.h"
#include "navigation.h"
#include "simplelocalization.h"

#include "RunningSequence.h"

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

MotorSpeed leftMotorSpeed(&leftEncoder, LEFTWHEEL_GEAR_RATIO);
MotorSpeed rightMotorSpeed(&rightEncoder, RIGHTWHEEL_GEAR_RATIO);

WheelPID leftPID;
WheelPID rightPID;

Logger logger(&sdBlockDevice, &littleFileSystem2);
Console console(&mu2, &logger);

WheelControl leftControl(&leftWheelMotor, &leftPID, &leftMotorSpeed);
WheelControl rightControl(&rightWheelMotor, &rightPID, &rightMotorSpeed);

FusionOdometry ekf;

SimpleLocalization simpleLocalization(&leftMotorSpeed, &rightMotorSpeed, BODY_LENGTH, WHEEL_RADIUS);
EKFLocalization localization(&leftMotorSpeed, &rightMotorSpeed, &imu, &ekf, BODY_LENGTH, WHEEL_RADIUS);

TorqueControl torqueControl(&leftMotorSpeed, &rightMotorSpeed, &leftControl, &rightControl, &leftPID, &rightPID);
Navigation navi(&localization, &leftControl, &rightControl, &torqueControl);

RunningSequence runningSequence(&navi, &localization, &torqueControl, &imu, &leftMotorSpeed, &rightMotorSpeed,
                                &leftControl, &rightControl, &console, &logger);

Thread speedThread(osPriorityAboveNormal, 1024, nullptr, nullptr);
Thread printThread(osPriorityAboveNormal, 1024, nullptr, nullptr);

void printThreadLoop() {
  while (true) {
    double tmp = imu.gyrX() * PI / 180.0;
    snprintf(printBuffer, PRINT_BUFFER_SIZE,
             "Ltsp:%f, Rtsp:%f, Lcsp:%f, Rcsp:%f, w_wh:%f, w_gy:%f, t_kf:%f, w_kf:%f, x_kf:%f, y_kf:%f, v_kf:%f, "
             "beta:%f, slip:%f\r\n",
             navi.leftTargetSpeed(), navi.rightTargetSpeed(), leftMotorSpeed.currentSpeedRPM(),
             rightMotorSpeed.currentSpeedRPM(), localization.getAngularVelocityFromWheelOdometry(), tmp,
             localization.theta(), localization.omega(), localization.x(), localization.y(), localization.v(),
             localization.beta(), localization.slip());
    bufferedSerial.write(printBuffer, strlen(printBuffer));
    ThisThread::sleep_for(500ms);
  }
}

// main() runs in its own thread in the OS
int main() {
  runningSequence.start(FIRST);
  if (imu.getStatus() == LSM9DS1_STATUS_SUCCESS_TO_CONNECT) {
    snprintf(printBuffer, PRINT_BUFFER_SIZE, "Succeeded connecting LSM9DS1.\r\n");
    // bufferedSerial.write(printBuffer,strlen(printBuffer));
  } else {
    snprintf(printBuffer, PRINT_BUFFER_SIZE, "Failed to connect LSM9DS1.\r\n");
    // bufferedSerial.write(printBuffer,strlen(printBuffer));
  }
  printThread.start(printThreadLoop);

  //ランニングシーケンスの開始
  runningSequence.start(FIRST);
  while (runningSequence.state() != ARRIVED_SECOND_POLE && runningSequence.state() != TERMINATE) {
    ThisThread::sleep_for(100ms);
  }
  runningSequence.stop();
  ThisThread::sleep_for(1s);

  runningSequence.start(SECOND);
  while (runningSequence.state() != ARRIVED_THIRD_POLE && runningSequence.state() != TERMINATE) {
    ThisThread::sleep_for(100ms);
  }
  runningSequence.stop();
  ThisThread::sleep_for(1s);

  runningSequence.start(THIRD);
  while (runningSequence.state() != ARRIVED_FOURTH_POLE && runningSequence.state() != TERMINATE) {
    ThisThread::sleep_for(100ms);
  }
  runningSequence.stop();
  if (runningSequence.state() == ARRIVED_FOURTH_POLE) {
    snprintf(printBuffer, PRINT_BUFFER_SIZE, "SUCCESS\r\n");
    bufferedSerial.write(printBuffer, strlen(printBuffer));
  }
  while (true) {
    ThisThread::sleep_for(1s);
  }
}
