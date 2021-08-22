#pragma once

#include "mbed.h"

#define PA1010D_I2C_ADDR 0x10U
#define PA1010D_I2C_WRITE_ADDR ((PA1010D_I2C_ADDR) << 1U)
#define PA1010D_I2C_READ_ADDR (((PA1010D_I2C_ADDR) << 1U) | 0x01U)

#define PA1010D_THREAD_PRIORITY osPriorityHigh
#define PA1010D_THREAD_STACK_SIZE 1024
#define PA1010D_THREAD_NAME "PA1010D"

#define PA1010D_BUFFER_SIZE 256

#define PA1010D_POLLING_PERIOD 100ms
#define PA1010D_READ_SIZE 255

#define PA1010D_BASE_10 10
#define PA1010D_BASE_16 16

#define PA1010D_CHAR_FIELD(ptr) (*(ptr) == ',' ? '\0' : *(ptr))

#define PA1010D_GNRMC_PREFIX "$GNRMC"
#define PA1010D_GNGGA_PREFIX "$GNGGA"

#define PA1010D_COMMAND_SIZE 64
#define PA1010D_COMMAND_SUFFIX_FORMAT "%02X\r\n"

// Packet Type: 220 PMTK_SET_NMEA_UPDATERATE
#define PA1010D_PMTK_SET_NMEA_UPDATERATE_FORMAT "$PMTK220,%u*"

// Before user input this command for update rate setting, it needs to see if the baud rate is enough or not.
// User can use PMTK251 command for baud rate setting
// 1000(millisecond) = 1(sec) 1/1 = 1Hz
// 200(millisecond) = 0.2(sec) 1/0.2 = 5 Hz
// 100(millisecond) = 0.1(sec) 1/0.1 = 10 Hz

// 出力形式が多いとレートをあげられない
// 出力形式を一時的に減らして設定することでレートをあげられるが、取りこぼすので望ましくない
// 10Hzでは1形式
#define PA1010D_UPDATERATE 100

// Packet Type: 225 PMTK_CMD_PERIODIC_MODE
#define PA1010D_PMTK_CMD_PERIODIC_MODE "$PMTK225,%u*"

// 動作モード
// ‘0’ = go back to normal mode
// ‘1’ = Periodic backup mode
// ‘2’ = Periodic standby mode
// ‘4’ = Perpetual mode(this mode need be work with relative hardware pin)
// ‘8’ = AlwaysLocateTM standby mode
// ‘9’ = AlwaysLocateTM backup mode
#define PA1010D_MODE 0

// Packet Type: 314 PMTK_API_SET_NMEA_OUTPUT
#define PA1010D_PMTK_API_SET_NMEA_OUTPUT_FORMAT "$PMTK314,0,%u,%u,%u,%u,%u,0,0,0,0,0,0,0,0,0,0,0,0,0*"

// 出力設定
// "0" - Disabled or not supported sentence
// "1" - Output once every one position fix
// "2" - Output once every two position fixes
// "3" - Output once every three position fixes
// "4" - Output once every four position fixes
// "5" - Output once every five position fixes
#define PA1010D_RMC_OUTPUT 0
#define PA1010D_VTG_OUTPUT 0
#define PA1010D_GGA_OUTPUT 1
#define PA1010D_GSA_OUTPUT 0
#define PA1010D_GSV_OUTPUT 0

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

  using GGAPacket = struct {
    double utc;                   // UTC時刻 hhmmss.sss
    double latitude;              // 緯度 ddmm.mmmm
    char nsIndicator;             // N: 北緯, S: 南緯
    double longitude;             // 経度: dddmm.mmmm
    char ewIndicator;             // E: 東経, W: 西経
    uint8_t positionFixIndicator; // 0: Fix not avalilable, 1: GPS fix, 2: Differencial GPS fix
    uint8_t satellitesUsed;       // 使用衛星数: Range 0 to 14
    float hdop;                   // 水平精度低下率
    float mslAltitude;            // アンテナの海抜高さ (m)
    float geoidalSeparation;      // ジオイド高 (m)
    float ageOfDiffCorr;          // DGPS データのエイジ (秒)
    uint16_t stationID;           // DGPS 局 ID (0000-1023)
  };

private:
  I2C *_i2c;
  unique_ptr<Thread> _thread;

  char _packetBuffer[PA1010D_BUFFER_SIZE];
  size_t _packetBufferPosition;

  RMCPacket _rmc;
  GGAPacket _gga;

  bool _isSetTime;

public:
private:
  void threadLoop();

  uint8_t checksum(const char *packet);

  // コンフィグの書き込み
  void setup();

  void pmtkCommand(const char *format, ...) MBED_PRINTF_METHOD(1, 2);

  // モジュールからの読み出し
  void polling();

  // NMEAのデコード
  void nmeaDecode();

  // RMCのデコード
  void rmcDecode();

  // GGAのデコード
  void ggaDecode();

  void setTime();

public:
  explicit PA1010D(I2C *i2c);

  void start();

  void stop();

  void getRMC(RMCPacket *rmc);

  void getGGA(GGAPacket *gga);
};
