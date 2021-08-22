#pragma once

#include "BlockDevice.h"
#include "FileSystem.h"
#include "mbed.h"

#define LOGGER_FORCE_REFORMAT false
#define LOGGER_MESSAGE_FILE_PATH "/MessageLog.txt"
#define LOGGER_GPS_FILE_PATH "/GPSLog.bin"

#define LOGGER_PRINTF_BUFFER_SIZE 256

class Logger {
private:
public:
#pragma pack(1)
  using GPSLogData = struct {
    double utc;                   // UTC時刻 hhmmss.sss
    double latitude;              // 緯度 ddmm.mmmm
    char nsIndicator;             // N: 北緯, S: 南緯
    double longitude;             // 経度: dddmm.mmmm
    char ewIndicator;             // E: 東経, W: 西経
    uint8_t positionFixIndicator; // 0: Fix not avalilable, 1: GPS fix, 2: Differencial GPS fix
  };
#pragma pack()

private:
  FileSystem *_fileSystem;
  BlockDevice *_blockDevice;

  // メッセージログ用のファイル
  shared_ptr<File> _messageFile;

  // GPSログ用のファイル
  shared_ptr<File> _gpsFile;

public:
private:
  void mount();

  shared_ptr<File> open(const char *path);

  void close(shared_ptr<File> file, const char *path);

  int write(shared_ptr<File> file, const void *buffer, size_t size);

  int read(shared_ptr<File> file, void *buffer, size_t size);

  void dumpMessageLog();

  void dumpGPSLog();

public:
  explicit Logger(BlockDevice *blockDevice, FileSystem *fileSystem);

  void init();

  void deinit();

  // メッセージログの書き込み
  int lprintf(const char *format, ...) MBED_PRINTF_METHOD(1, 2);

  int vlprintf(const char *format, va_list ap) MBED_PRINTF_METHOD(1, 0);

  // GPSログの書き込み
  void gpsLog(GPSLogData *data);
};
