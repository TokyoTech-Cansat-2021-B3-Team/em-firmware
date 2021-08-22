#include "PA1010D.h"

PA1010D::PA1010D(I2C *i2c)
    : _i2c(i2c), _thread(), _queue(), _packetBuffer(), _packetBufferPosition(0), _rmc(), _gga(), _isSetTime(false) {}

void PA1010D::start() {
  if (!_thread) {
    _thread = make_unique<Thread>(PA1010D_THREAD_PRIORITY,   //
                                  PA1010D_THREAD_STACK_SIZE, //
                                  nullptr,                   //
                                  PA1010D_THREAD_NAME);
    _thread->start(callback(this, &PA1010D::threadLoop));
  }
}

void PA1010D::stop() {
  if (_thread) {
    _thread->terminate();
    _thread.reset();
  }
}

void PA1010D::threadLoop() {
  setup();

  _queue.call_every(PA1010D_POLLING_PERIOD, [this]() {
    polling();

    setTime();
  });

  _queue.dispatch_forever();
}

uint8_t PA1010D::checksum(const char *packet) {
  uint8_t ret = 0;

  // チェックサム
  const char *beginPtr = strchr(packet, '$');
  const char *endPtr = strchr(packet, '*');
  const char *ptr = beginPtr + 1;

  if (beginPtr == nullptr || //
      endPtr == nullptr ||   //
      strlen(packet) <= 1) {
    // printf("error format\n");
    return 0;
  }

  while (ptr < endPtr) {
    ret ^= *ptr;
    ptr++;
  }

  return ret;
}

void PA1010D::setup() {
  // 空読みしてバッファクリア
  char readBuffer[PA1010D_READ_SIZE];

  _i2c->read(PA1010D_I2C_READ_ADDR, //
             readBuffer,            //
             PA1010D_READ_SIZE);

  // 動作モード
  pmtkCommand(PA1010D_PMTK_CMD_PERIODIC_MODE, PA1010D_MODE);

  // 出力形式
  pmtkCommand(PA1010D_PMTK_API_SET_NMEA_OUTPUT_FORMAT,                    //
              PA1010D_RMC_OUTPUT, PA1010D_VTG_OUTPUT, PA1010D_GGA_OUTPUT, //
              PA1010D_GSA_OUTPUT, PA1010D_GSV_OUTPUT);

  // 出力レート
  // 出力形式を少なくしないとレートをあげられない
  pmtkCommand(PA1010D_PMTK_SET_NMEA_UPDATERATE_FORMAT, PA1010D_UPDATERATE);
}

void PA1010D::pmtkCommand(const char *format, ...) {

  va_list ap;
  va_start(ap, format);

  char commandBuffer[PA1010D_COMMAND_SIZE];

  // コマンド
  vsnprintf(commandBuffer, PA1010D_COMMAND_SIZE, format, ap);

  // チェックサム + 改行
  uint8_t cs = checksum(commandBuffer);
  snprintf(commandBuffer + strlen(commandBuffer), PA1010D_COMMAND_SIZE - strlen(commandBuffer),
           PA1010D_COMMAND_SUFFIX_FORMAT, cs);

  // 送信
  _i2c->write(PA1010D_I2C_WRITE_ADDR, commandBuffer, strlen(commandBuffer));

  va_end(ap);
};

void PA1010D::polling() {
  char readBuffer[PA1010D_READ_SIZE];

  // モジュールから読み出し
  if (_i2c->read(PA1010D_I2C_READ_ADDR, //
                 readBuffer,            //
                 PA1010D_READ_SIZE) != 0) {
    return;
  };

  size_t i = 0;
  for (i = 0; i < PA1010D_READ_SIZE; i++) {
    // NMEAパケットの終端
    if (readBuffer[i] == '\r') {
      // 文字列化
      _packetBuffer[_packetBufferPosition] = '\0';
      _packetBufferPosition++;

      // パケットの解析
      nmeaDecode();

      // バッファ消去
      _packetBufferPosition = 0;

      // 次のLFは読み飛ばす
      i++;
    }
    // バッファの終端
    else if (readBuffer[i] == '\n') {
      // 読み出し終了
      break;
    }
    // 通常のデータ
    else {
      // 文字列化用に1バイト残してサイズチェック
      if (_packetBufferPosition < PA1010D_BUFFER_SIZE - 1) {
        // packetBufferへ移動
        _packetBuffer[_packetBufferPosition] = readBuffer[i];
        _packetBufferPosition++;
      } else {
        // パケットがバッファサイズを超える場合は超過分を無視
        continue;
      }
    }
  }
}

void PA1010D::nmeaDecode() {
  // 表示
  //   printf("%s\n", _packetBuffer);

  // チェックサム
  uint8_t calChecksum = checksum(_packetBuffer);
  uint8_t trueChecksum = strtol(_packetBuffer + strlen(_packetBuffer) - 2, nullptr, PA1010D_BASE_16);

  if (calChecksum != trueChecksum) {
    // 不正なパケット
    // printf("error checksum\n");
    return;
  }

  // パケットを分類してデコード
  if (strstr(_packetBuffer, PA1010D_GNRMC_PREFIX) == _packetBuffer) {
    rmcDecode();
  } else if (strstr(_packetBuffer, PA1010D_GNGGA_PREFIX) == _packetBuffer) {
    ggaDecode();
  }
}

void PA1010D::rmcDecode() {
  _rmc = {0};

  // UTC
  char *ptr = nullptr;
  if ((ptr = strchr(_packetBuffer, ',')) == nullptr) {
    return;
  }
  _rmc.utc = strtod(ptr + 1, nullptr);

  // STATUS
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _rmc.status = PA1010D_CHAR_FIELD(ptr + 1);

  // LAT
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _rmc.latitude = strtod(ptr + 1, nullptr);

  // NS
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _rmc.nsIndicator = PA1010D_CHAR_FIELD(ptr + 1);

  // LNG
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _rmc.longitude = strtod(ptr + 1, nullptr);

  // EW
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _rmc.ewIndicator = PA1010D_CHAR_FIELD(ptr + 1);

  // SOG
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _rmc.speedOverGround = strtof(ptr + 1, nullptr);

  // COG
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _rmc.courseOverGround = strtof(ptr + 1, nullptr);

  // DATE
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _rmc.date = strtoul(ptr + 1, nullptr, PA1010D_BASE_10);

  // MV
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    printf("return \n");
    return;
  }
  _rmc.magneticVariation = strtof(ptr + 1, nullptr);

  // VD
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _rmc.variationDirection = PA1010D_CHAR_FIELD(ptr + 1);

  // MODE
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _rmc.mode = PA1010D_CHAR_FIELD(ptr + 1);

  // 表示
  //   printf("utc: %lf\n", _rmc.utc);
  //   printf("status: %c\n", _rmc.status);
  //   printf("latitude: %lf\n", _rmc.latitude);
  //   printf("nsIndicator: %c\n", _rmc.nsIndicator);
  //   printf("longitude: %lf\n", _rmc.longitude);
  //   printf("ewIndicator: %c\n", _rmc.ewIndicator);
  //   printf("speedOverGround: %f\n", _rmc.speedOverGround);
  //   printf("courseOverGround: %f\n", _rmc.courseOverGround);
  //   printf("date: %u\n", _rmc.date);
  //   printf("magneticVariation: %f\n", _rmc.magneticVariation);
  //   printf("variationDirection: %c\n", _rmc.variationDirection);
  //   printf("mode: %c\n", _rmc.mode);
}

void PA1010D::ggaDecode() {
  _gga = {0};

  // UTC
  char *ptr = nullptr;
  if ((ptr = strchr(_packetBuffer, ',')) == nullptr) {
    return;
  }
  _gga.utc = strtod(ptr + 1, nullptr);

  // LAT
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _gga.latitude = strtod(ptr + 1, nullptr);

  // NS
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _gga.nsIndicator = PA1010D_CHAR_FIELD(ptr + 1);

  // LNG
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _gga.longitude = strtod(ptr + 1, nullptr);

  // EW
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _gga.ewIndicator = PA1010D_CHAR_FIELD(ptr + 1);

  // FIX
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _gga.positionFixIndicator = strtoul(ptr + 1, nullptr, PA1010D_BASE_10);

  // SU
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _gga.satellitesUsed = strtoul(ptr + 1, nullptr, PA1010D_BASE_10);

  // HDOP
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    printf("return \n");
    return;
  }
  _gga.hdop = strtof(ptr + 1, nullptr);

  // MSLALT
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    printf("return \n");
    return;
  }
  _gga.mslAltitude = strtof(ptr + 1, nullptr);

  // GEOSEP
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    printf("return \n");
    return;
  }
  _gga.geoidalSeparation = strtof(ptr + 1, nullptr);

  // AGE
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    printf("return \n");
    return;
  }
  _gga.ageOfDiffCorr = strtof(ptr + 1, nullptr);

  // STATION
  if ((ptr = strchr(ptr + 1, ',')) == nullptr) {
    return;
  }
  _gga.stationID = strtoul(ptr + 1, nullptr, PA1010D_BASE_10);

  // 表示
  //   printf("utc: %lf\n", _gga.utc);
  //   printf("latitude: %lf\n", _gga.latitude);
  //   printf("nsIndicator: %c\n", _gga.nsIndicator);
  //   printf("longitude: %lf\n", _gga.longitude);
  //   printf("ewIndicator: %c\n", _gga.ewIndicator);
  //   printf("positionFixIndicator: %u\n", _gga.positionFixIndicator);
  //   printf("satellitesUsed: %u\n", _gga.satellitesUsed);
  //   printf("hdop: %f\n", _gga.hdop);
  //   printf("mslAltitude: %f\n", _gga.mslAltitude);
  //   printf("geoidalSeparation: %f\n", _gga.geoidalSeparation);
  //   printf("ageOfDiffCorr: %f\n", _gga.ageOfDiffCorr);
  //   printf("stationID: %u\n", _gga.stationID);
}

void PA1010D::setTime() {
  if (_isSetTime) {
    if (PA1010D_RMC_OUTPUT && _rmc.status == 'A') {
      tm t = {
          static_cast<int>(_rmc.utc) % 100,                // sec
          static_cast<int>(_rmc.utc / 100) % 100,          // min
          static_cast<int>(_rmc.utc / 10000) % 100,        // hour
          static_cast<int>(_rmc.date / 10000) % 100,       // day
          static_cast<int>(_rmc.date / 100) % 100 - 1,     // month
          static_cast<int>(_rmc.date) % 100 + 2000 - 1900, // year
      };
      set_time(mktime(&t));
      _isSetTime = true;
    }
    // else if (PA1010D_GGA_OUTPUT && _gga.positionFixIndicator > 0) {
    //     tm t = {
    //         static_cast<int>(_gga.utc) % 100,         // sec
    //         static_cast<int>(_gga.utc / 100) % 100,   // min
    //         static_cast<int>(_gga.utc / 10000) % 100, // hour
    //         1,
    //         1 - 1,
    //         1970 - 1900,
    //     };
    //     //   printf("%d,%d,%d,%d,%d,%d\n", t.tm_sec, t.tm_min, t.tm_hour, t.tm_mday, t.tm_mon, t.tm_year);
    //     //   printf("%lf\n", _gga.utc);
    //     set_time(mktime(&t));
    //     _isSetTime = true;
    //   }
  }
}

void PA1010D::getRMC(RMCPacket *rmc) {
  if (rmc != nullptr) {
    *rmc = _rmc;
  }
}

void PA1010D::getGGA(GGAPacket *gga) {
  if (gga != nullptr) {
    *gga = _gga;
  }
}
