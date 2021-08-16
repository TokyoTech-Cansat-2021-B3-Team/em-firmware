#pragma once

#include "mbed.h"

#define PA1010D_I2C_ADDR 0x10U
#define PA1010D_I2C_WRITE_ADDR ((PA1010D_I2C_ADDR) << 1U)
#define PA1010D_I2C_READ_ADDR (((PA1010D_I2C_ADDR) << 1U) | 0x01U)

#define PA1010D_THREAD_PRIORITY osPriorityHigh
#define PA1010D_THREAD_STACK_SIZE 1024
#define PA1010D_THREAD_NAME "PA1010D"

#define PA1010D_BUFFER_SIZE 256

#define PA1010D_POLLING_PERIOD 500ms
#define PA1010D_READ_SIZE 255

#define PA1010D_BASE_10 10
#define PA1010D_BASE_16 16

#define PA1010D_CHAR_FIELD(ptr) (*(ptr) == ',' ? '\0' : *(ptr))

#define PA1010D_GNRMC_ID "GNRMC"

class PA1010D {
private:
public:
  using RMCPacket = struct {
    double utc;              // UTC時刻 hhmmss.sss
    char status;             // A: 有効, V: 無効
    double latitude;         // 緯度 ddmm.mmmm
    char nsIndicator;        // N: 北緯, S: 南緯
    double longitude;        // 経度: dddmm.mmmm
    char ewIndicator;        // E: 東経, W: 西経
    float speedOverGround;   // 対地速度 knots
    float courseOverGround;  // 対地コース degrees
    uint32_t date;           // 日付 ddmmyy
    float magneticVariation; // 磁気偏角 degrees
    char variationDirection; // 磁気偏角の向き E: 東, W: 西
    char mode;               // N: データなし, A: 単独測位, D: DGPS, E: Dead Reckoning
  };

private:
  I2C *_i2c;
  unique_ptr<Thread> _thread;

  char _packetBuffer[PA1010D_BUFFER_SIZE];
  size_t _packetBufferPosition;

  RMCPacket _rmc;

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

  void getRMC(RMCPacket *rmc);
};
