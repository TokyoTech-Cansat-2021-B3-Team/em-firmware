#include "PA1010D.h"

PA1010D::PA1010D(I2C *i2c) : _i2c(i2c), _thread(), _packetBuffer(), _packetBufferPosition(0), _rmc() {}

void PA1010D::start() {
  _thread = make_unique<Thread>(PA1010D_THREAD_PRIORITY,   //
                                PA1010D_THREAD_STACK_SIZE, //
                                nullptr,                   //
                                PA1010D_THREAD_NAME);
  _thread->start(callback(this, &PA1010D::threadLoop));
}

void PA1010D::stop() {
  _thread->terminate();
  _thread.reset();
}

void PA1010D::threadLoop() {
  while (true) {
    polling();

    ThisThread::sleep_for(PA1010D_POLLING_PERIOD);
  }
}

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
  char *beginPtr = strchr(_packetBuffer, '$');
  char *endPtr = strchr(_packetBuffer, '*');

  if (beginPtr == nullptr || //
      endPtr == nullptr ||   //
      strlen(_packetBuffer) <= 1) {
    // printf("error format\n");
    return;
  }

  char *ptr = beginPtr + 1;
  uint8_t calChecksum = 0;

  while (ptr < endPtr) {
    calChecksum ^= *ptr;
    ptr++;
  }

  uint8_t trueChecksum = strtol(endPtr + 1, nullptr, PA1010D_CHECKSUM_BASE);

  if (calChecksum != trueChecksum) {
    // 不正なパケット
    // printf("error checksum\n");
    return;
  }

  // パケットを分類してデコード
  if (strstr(_packetBuffer, PA1010D_GNRMC_ID) == _packetBuffer) {
    rmcDecode();
  }
}

void PA1010D::rmcDecode() {
  _rmc = {0};

  sscanf(_packetBuffer,
         PA1010D_GNRMC_ID ","      //
         PA1010D_GNRMC_UTC ","     //
         PA1010D_GNRMC_STATUS ","  //
         PA1010D_GNRMC_LAT ","     //
         PA1010D_GNRMC_NS ","      //
         PA1010D_GNRMC_LNG ","     //
         PA1010D_GNRMC_EW ","      //
         PA1010D_GNRMC_SOG ","     //
         PA1010D_GNRMC_COG ","     //
         PA1010D_GNRMC_DATE ","    //
         PA1010D_GNRMC_MV ","      //
         PA1010D_GNRMC_VD ","      //
         PA1010D_GNRMC_MODE "%*s", //
         &_rmc.utc,                //
         &_rmc.status,             //
         &_rmc.latitude,           //
         &_rmc.nsIndicator,        //
         &_rmc.longitude,          //
         &_rmc.ewIndicator,        //
         &_rmc.speedOverGround,    //
         &_rmc.courseOverGround,   //
         &_rmc.date,               //
         &_rmc.magneticVariation,  //
         &_rmc.variationDirection, //
         &_rmc.mode);

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

void PA1010D::getRMC(NmeaRmcPacket *rmc) {
  if (rmc != nullptr) {
    *rmc = _rmc;
  }
}
