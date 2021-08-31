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

// objects

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

// 着地検知シーケンス
void syncLandingSequence() {
  console.log("main", "Landing Sequence Sync Start");

  landingSequence.start();

  while (true) {
    // 正常終了
    if (landingSequence.state() == LandingSequence::Complete) {
      console.log("main", "Landing Sequence Complete");
      break;
    }

    // タイムアウト
    if (landingSequence.state() == LandingSequence::SequenceTimeout) {
      console.log("main", "Landing Sequence Timeout");
      break;
    }

    Stabilize stabilize(&imu, &leftWheelMotor, &rightWheelMotor);

    void printThreadLoop() {
      while (true) {
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "$%f %f;\r\n", stabilize.currentTheta(), stabilize.currentOutput());
        serial.write(printBuffer, strlen(printBuffer));
        ThisThread::sleep_for(20ms);
      }
      if (runningSequence.state() == TERMINATE) {
        runningSequence.stop();
        console.log("main", "Running Sequence Terminate");
        break;
      }

      ThisThread::sleep_for(1s);
    }
  }

  // main() runs in its own thread in the OS
  int main() {

    // ステッピングモータへの電源供給OFF
    loadingMotor.idleCurrent(false);
    imu.start();
    ThisThread::sleep_for(100ms);
    if (imu.getStatus() == LSM9DS1_STATUS_SUCCESS_TO_CONNECT) {
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "Succeeded connecting LSM9DS1.\r\n");
      serial.write(printBuffer, strlen(printBuffer));
    } else {
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "Failed to connect LSM9DS1.\r\n");
      serial.write(printBuffer, strlen(printBuffer));
    }

    stabilize.start();
    printThread.start(printThreadLoop);
    while (true) {
      ThisThread::sleep_for(100ms);
    }
  }
