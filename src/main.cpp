#include "mbed.h"

// includes
#include "PinAssignment.h"

// embedded
#include "LittleFileSystem2.h"
#include "SDBlockDevice.h"

// drivers
#include "Fusing.h"
#include "MU2.h"

// middlewares
#include "Console.h"
#include "Logger.h"

// sequences
#include "FusingSequence.h"

// defines
#define SPI_FREQUENCY 25000000
#define I2C_FREQUENCY 400000

// objects

// embedded
PwmOut fuseGate(FUSE_GATE);

I2C i2c(I2C_SDA, I2C_SCL);
BufferedSerial bufferedSerial(UART_TX, UART_RX, MU2_SERIAL_BAUDRATE);
SDBlockDevice sdBlockDevice(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL, SPI_FREQUENCY);
LittleFileSystem2 littleFileSystem2(nullptr);

// drivers
MU2 mu2(&bufferedSerial);
Fusing fusing(&fuseGate);

// middlewares
Logger logger(&sdBlockDevice, &littleFileSystem2);
Console console(&mu2, &logger);

// sequences
FusingSequence fusingSequence(&fusing, &console);

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

// main() runs in its own thread in the OS
int main() {
  // I2C速度変更
  i2c.frequency(I2C_FREQUENCY);

  fusingSequenceSyncStart();

  while (true) {
    ThisThread::sleep_for(1s);
  }
}
