#include "mbed.h"

// includes
#include "PinAssignment.h"

// embedded
#include "LittleFileSystem2.h"
#include "SDBlockDevice.h"

// drivers
#include "DCMotor.h"
#include "DrillMotor.h"
#include "MU2.h"
#include "QEI.h"
#include "Stepper.h"

// middlewares
#include "Console.h"
#include "Logger.h"

// sequences
#include "ProbeSequence.h"

// defines
#define SPI_FREQUENCY 25000000
#define I2C_FREQUENCY 400000

// objects

// embedded
PwmOut motor3In1(M3_IN1);
PwmOut motor3In2(M3_IN2);
PwmOut motor4In1(M4_IN1);
DigitalOut motor5Enable(M5_ENABLE);
DigitalOut motor5Step(M5_STEP);

I2C i2c(I2C_SDA, I2C_SCL);
BufferedSerial bufferedSerial(UART_TX, UART_RX, MU2_SERIAL_BAUDRATE);
SDBlockDevice sdBlockDevice(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL, SPI_FREQUENCY);
LittleFileSystem2 littleFileSystem2(nullptr);

// drivers
MU2 mu2(&bufferedSerial);

DCMotor verticalMotor(&motor3In1, &motor3In2);
DrillMotor drillMotor(&motor4In1);
Stepper loadingMotor(&motor5Step, &motor5Enable);
QEI verticalEncoder(ENC3_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

// middlewares
Logger logger(&sdBlockDevice, &littleFileSystem2);
Console console(&mu2, &logger);

// sequences
ProbeSequence probeSequence(&drillMotor, &verticalMotor, &loadingMotor, &verticalEncoder, &console);

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

// main() runs in its own thread in the OS
int main() {
  // I2C速度変更
  i2c.frequency(I2C_FREQUENCY);

  // probeSequence.test();

  probeSequenceSyncStart(ProbeSequence::Probe1);

  probeSequenceSyncStart(ProbeSequence::Probe2);

  probeSequenceSyncStart(ProbeSequence::Probe3);

  probeSequenceSyncStart(ProbeSequence::Probe4);

  while (true) {
    ThisThread::sleep_for(1s);
  }
}
