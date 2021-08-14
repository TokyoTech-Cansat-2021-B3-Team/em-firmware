#pragma once

#include "mbed.h"

#include "NmeaPacket.h"

#define PA1010D_I2C_ADDR 0x10U
#define PA1010D_I2C_WRITE_ADDR ((PA1010D_I2C_ADDR) << 1U)
#define PA1010D_I2C_READ_ADDR (((PA1010D_I2C_ADDR) << 1U) | 0x01U)

#define PA1010D_THREAD_PRIORITY osPriorityHigh
#define PA1010D_THREAD_STACK_SIZE 1024
#define PA1010D_THREAD_NAME "PA1010D"

#define PA1010D_BUFFER_SIZE 256

#define PA1010D_POLLING_PERIOD 500ms
#define PA1010D_READ_SIZE 255

#define PA1010D_CHECKSUM_BASE 16

#define PA1010D_GNRMC_ID "$GNRMC"
#define PA1010D_GNRMC_UTC "%lf"
#define PA1010D_GNRMC_STATUS "%c"
#define PA1010D_GNRMC_LAT "%lf"
#define PA1010D_GNRMC_NS "%c"
#define PA1010D_GNRMC_LNG "%lf"
#define PA1010D_GNRMC_EW "%c"
#define PA1010D_GNRMC_SOG "%f"
#define PA1010D_GNRMC_COG "%f"
#define PA1010D_GNRMC_DATE "%u"
#define PA1010D_GNRMC_MV "%f"
#define PA1010D_GNRMC_VD "%c"
#define PA1010D_GNRMC_MODE "%c"
#define PA1010D_GNRMC_CHECKSUM "%2s"

class PA1010D {
private:
  I2C *_i2c;
  unique_ptr<Thread> _thread;

  char _packetBuffer[PA1010D_BUFFER_SIZE];
  size_t _packetBufferPosition;

  NmeaRmcPacket _rmc;

public:
private:
  void threadLoop();

  // モジュールからの読み出し
  void polling();

  // NMEAのデコード
  void nmeaDecode();

  // RMCのデコード
  void rmcDecode();

public:
  explicit PA1010D(I2C *i2c);

  void start();

  void stop();

  void getRMC(NmeaRmcPacket *rmc);
};
