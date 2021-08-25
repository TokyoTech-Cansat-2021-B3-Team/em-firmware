#include "mbed.h"

// includes
#include "PinAssignment.h"

// embedded
#include "LittleFileSystem2.h"
#include "SDBlockDevice.h"

// drivers
#include "Fusing.h"
#include "MU2.h"
#include "PA1010D.h"

// middlewares
#include "Console.h"
#include "Logger.h"

// sequences
#include "GPSDownlink.h"
#include "FusingSequence.h"
#include "LandingSequence.h"

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
Fusing fusing(&fuseGate);

PA1010D pa1010d(&i2c);
BME280 bme280(&i2c);

MU2 mu2(&bufferedSerial);

// middlewares
Logger logger(&sdBlockDevice, &littleFileSystem2);
Console console(&mu2, &logger);

Variometer variometer(&bme280);

// sequences
GPSDownlink gpsDownlink(&pa1010d, &console, &logger);
LandingSequence landingSequence(&variometer, &console);
FusingSequence fusingSequence(&fusing, &console);

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

// main() runs in its own thread in the OS
int main() {
  // I2C速度変更
  i2c.frequency(I2C_FREQUENCY);

  // GPSダウンリンク
  gpsDownlink.start();

  // 着地検知シーケンス
  syncLandingSequence();

  // パラシュート分離シーケンス
  fusingSequenceSyncStart();

  console.log("main", "All Sequence Complete");
  console.log("main", "Start Sleep Forever");

  while (true) {
    ThisThread::sleep_for(1s);
  }
}
