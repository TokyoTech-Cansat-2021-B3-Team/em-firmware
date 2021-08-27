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
PwmOut motor3In1(M3_IN1);
PwmOut motor3In2(M3_IN2);
PwmOut motor4In1(M4_IN1);
DigitalOut motor5Enable(M5_ENABLE);
DigitalOut motor5Step(M5_STEP);
PwmOut fuseGate(FUSE_GATE);

I2C i2c(I2C_SDA, I2C_SCL);
BufferedSerial bufferedSerial(UART_TX, UART_RX, MU2_SERIAL_BAUDRATE);

SDBlockDevice sdBlockDevice(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL, SPI_FREQUENCY);
LittleFileSystem2 littleFileSystem2(nullptr);

// drivers
WheelMotor leftWheelMotor(&motor1In1, &motor1In2);
WheelMotor rightWheelMotor(&motor2In1, &motor2In2);
QEI leftEncoder(ENC1_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);
QEI rightEncoder(ENC2_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

Fusing fusing(&fuseGate);
DCMotor verticalMotor(&motor3In1, &motor3In2);
DrillMotor drillMotor(&motor4In1);
Stepper loadingMotor(&motor5Step, &motor5Enable);
QEI verticalEncoder(ENC3_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

PA1010D pa1010d(&i2c);
BME280 bme280(&i2c);
MU2 mu2(&bufferedSerial);
LSM9DS1 imu(&i2c);

// middlewares
MotorSpeed leftMotorSpeed(&leftEncoder, 1000.0);
MotorSpeed rightMotorSpeed(&rightEncoder, 1000.0);
WheelPID leftPID;
WheelPID rightPID;
WheelControl leftControl(&leftWheelMotor, &leftPID, &leftMotorSpeed);
WheelControl rightControl(&rightWheelMotor, &rightPID, &rightMotorSpeed);
FusionOdometry ekf;
Localization localization(&leftMotorSpeed, &rightMotorSpeed, &imu, &ekf, 180.0e-3, 68.0e-3);
Navigation navi(&localization, &leftControl, &rightControl);

Logger logger(&sdBlockDevice, &littleFileSystem2);
Console console(&mu2, &logger);
Variometer variometer(&bme280);

// sequences
GPSDownlink gpsDownlink(&pa1010d, &console, &logger);
LandingSequence landingSequence(&variometer, &console);
FusingSequence fusingSequence(&fusing, &console);
ProbeSequence probeSequence(&drillMotor, &verticalMotor, &loadingMotor, &verticalEncoder, &console);
RunningSequence runningSequence(&navi, &localization, &imu, &leftMotorSpeed, &rightMotorSpeed, &leftControl,
                                &rightControl, &console, &logger);

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

    ThisThread::sleep_for(1s);
  }

  landingSequence.stop();
}

// パラシュート分離シーケンス
void fusingSequenceSyncStart() {
  console.log("main", "Fusing Sequence Sync Start");

  fusingSequence.start();

  // 正常終了
  // フィードバックなしなので必ず起こる
  while (fusingSequence.state() != FusingSequence::Complete) {
    ThisThread::sleep_for(1s);
  }

  console.log("main", "Fusing Sequence Complete");

  fusingSequence.stop();
}

// 刺し込みシーケンス
void probeSequenceSyncStart(ProbeSequence::ProbeNumber number) {
  console.log("main", "Probe%d Sequence Sync Start", number);

  probeSequence.start(number);

  // 正常終了
  // 内部でタイムアウトして終了するので必ずCompleteになる
  while (probeSequence.state() != ProbeSequence::Complete) {
    ThisThread::sleep_for(1s);
  }

  console.log("main", "Probe%d Sequence Complete", number);

  probeSequence.stop();
}

void runningSequenceSyncStart(RunningSqequneceType type) {

  runningSequence.start(type);

  if (imu.getStatus() == LSM9DS1_STATUS_SUCCESS_TO_CONNECT) {
    console.log("main", "Succeeded connecting LSM9DS1.\n");
  } else {
    console.log("main", "Failed to connect LSM9DS1.\n");
  }

  while (true) {
    if (runningSequence.state() == ARRIVED_SECOND_POLE) {
      runningSequence.stop();
      console.log("main", "Arrived Seconds Pole");
      break;
    }
    if (runningSequence.state() == ARRIVED_THIRD_POLE) {
      runningSequence.stop();
      console.log("main", "Arrived Third Pole");
      break;
    }
    if (runningSequence.state() == ARRIVED_FOURTH_POLE) {
      runningSequence.stop();
      console.log("main", "Arrived Fourth Pole");
      break;
    }
    if (runningSequence.state() == TERMINATE) {
      runningSequence.stop();
      console.log("main", "Running Sequence Terminate");
      break;
    }
  }
}

// main() runs in its own thread in the OS
int main() {
  //   verticalMotor.forward(1.0);

  //   while (true) {
  //     ThisThread::sleep_for(1s);
  //   }

  // ダウンリンクとログの有効化
  console.init();

  console.log("main", "Initialize Program !!");

  // I2C速度変更
  i2c.frequency(I2C_FREQUENCY);

  // ステッピングモータへの電源供給OFF
  loadingMotor.idleCurrent(false);

  // GPSダウンリンク
  gpsDownlink.start();

  // 着地検知シーケンス
  syncLandingSequence();

  // パラシュート分離シーケンス
  fusingSequenceSyncStart();

  // 刺し込みシーケンス1
  //   probeSequenceSyncStart(ProbeSequence::Probe1);

  // 走行シーケンス1
  runningSequenceSyncStart(FIRST);

  // 刺し込みシーケンス2
  //   probeSequenceSyncStart(ProbeSequence::Probe2);

  // 走行シーケンス2
  runningSequenceSyncStart(SECOND);

  // 刺し込みシーケンス3
  //   probeSequenceSyncStart(ProbeSequence::Probe3);

  // 走行シーケンス3
  runningSequenceSyncStart(THIRD);

  // 刺し込みシーケンス4
  //   probeSequenceSyncStart(ProbeSequence::Probe4);

  console.log("main", "All Sequence Complete");
  console.log("main", "Start Sleep Forever");

  while (true) {
    ThisThread::sleep_for(1s);
  }
}
