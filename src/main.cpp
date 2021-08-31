#include "mbed.h"

// includes
#include "PinAssignment.h"

// embedded
#include "LittleFileSystem2.h"
#include "SDBlockDevice.h"

// drivers
#include "DCMotor.h"
#include "DrillMotor.h"
#include "Fusing.h"
#include "MU2.h"
#include "QEI.h"
#include "Stepper.h"
#include "lsm9ds1.h"
// middlewares
#include "MotorSpeed.h"
#include "WheelControl.h"
#include "WheelMotor.h"
#include "WheelPID.h"
#include "fusion-odometry.h"
#include "localization.h"
#include "navigation.h"
#include "stabilize.h"

#include "Console.h"
#include "Logger.h"

// sequences
#include "FusingSequence.h"
#include "GPSDownlink.h"
#include "LandingSequence.h"
#include "ProbeSequence.h"
#include "RunningSequence.h"

// defines
#define SPI_FREQUENCY 25000000
#define I2C_FREQUENCY 400000
#define PRINT_BUFFER_SIZE 128

// objects
char printBuffer[PRINT_BUFFER_SIZE];

// embedded
PwmOut motor1In1(M1_IN1);
PwmOut motor1In2(M1_IN2);
PwmOut motor2In1(M2_IN1);
PwmOut motor2In2(M2_IN2);
DigitalOut motor5Enable(M5_ENABLE);
DigitalOut motor5Step(M5_STEP);
PwmOut fuseGate(FUSE_GATE);

I2C i2c(I2C_SDA, I2C_SCL);
BufferedSerial bufferedSerial(UART_TX, UART_RX, MU2_SERIAL_BAUDRATE);

// drivers
Stepper loadingMotor(&motor5Step, &motor5Enable);
WheelMotor leftWheelMotor(&motor1In1, &motor1In2);
WheelMotor rightWheelMotor(&motor2In1, &motor2In2);
QEI leftEncoder(ENC1_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);
QEI rightEncoder(ENC2_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

Fusing fusing(&fuseGate);

LSM9DS1 imu(&i2c);

// middlewares
MotorSpeed leftMotorSpeed(&leftEncoder, 1000.0);
MotorSpeed rightMotorSpeed(&rightEncoder, 1000.0);
WheelPID leftPID;
WheelPID rightPID;
WheelControl leftControl(&leftWheelMotor, &leftPID, &leftMotorSpeed);
WheelControl rightControl(&rightWheelMotor, &rightPID, &rightMotorSpeed);

Thread printTask(osPriorityRealtime, 512, nullptr, nullptr);

Stabilize stabilize(&imu, &leftWheelMotor, &rightWheelMotor);

void printThreadLoop() {
  while (true) {
    snprintf(printBuffer, PRINT_BUFFER_SIZE, "$%f %f;\r\n", stabilize.currentTheta(), stabilize.currentOutput());
    bufferedSerial.write(printBuffer, strlen(printBuffer));
    ThisThread::sleep_for(20ms);
  }

  ThisThread::sleep_for(1s);
  }

  // main() runs in its own thread in the OS
  int main() {

    // ステッピングモータへの電源供給OFF
    loadingMotor.idleCurrent(false);
    imu.start();
    ThisThread::sleep_for(100ms);
    if (imu.getStatus() == LSM9DS1_STATUS_SUCCESS_TO_CONNECT) {
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "Succeeded connecting LSM9DS1.\r\n");
      bufferedSerial.write(printBuffer, strlen(printBuffer));
    } else {
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "Failed to connect LSM9DS1.\r\n");
      bufferedSerial.write(printBuffer, strlen(printBuffer));
    }

    stabilize.start();
    printTask.start(printThreadLoop);
    while (true) {
      ThisThread::sleep_for(100ms);
    }
  }
