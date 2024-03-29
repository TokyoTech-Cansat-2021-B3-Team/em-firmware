#pragma once

#include "BlockDevice.h"
#include "FileSystem.h"
#include "mbed.h"
#include <memory>

#define LOGGER_FORCE_REFORMAT false
#define LOGGER_MOUNT_POINT "fs"
#define LOGGER_DIRECTORY "/"
#define LOGGER_MESSAGE_FILE_NAME_FORMAT "MessageLog%03u.txt"
#define LOGGER_GPS_FILE_NAME_FORMAT "GPSLog%03u.bin"
#define LOGGER_RUNNING_FILE_NAME_FORMAT "RunningLog%03u.bin"

#define LOGGER_FILE_PATH_BUFFER_SIZE 256
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
  using RunningData = struct {
    double x;
    double y;
    double theta;
    double leftTargetSpeed;
    double rightTargetSpeed;
    double leftSpeed;
    double rightSpeed;
  };
#pragma pack()

private:
  FileSystem *_fileSystem;
  BlockDevice *_blockDevice;

  // メッセージログ用のファイル
  shared_ptr<File> _messageFile;

  // GPSログ用のファイル
  shared_ptr<File> _gpsFile;

  // 走行シーケンスログ用のファイル
  shared_ptr<File> _runningFile;

  bool _isInit;

  // ファイルのID
  size_t id;

public:
private:
  void mount();

  shared_ptr<File> open(const char *path);

  void close(shared_ptr<File> file, const char *path);

  int write(shared_ptr<File> file, const void *buffer, size_t size);

  int read(shared_ptr<File> file, void *buffer, size_t size);

  int getLastID();

  void dumpMessageLog(size_t id);

  void dumpGPSLog(size_t id);

  void dumpRunningLog(size_t id);

public:
  explicit Logger(BlockDevice *blockDevice, FileSystem *fileSystem);

  void init();

  void deinit();

  // メッセージログの書き込み
  int lprintf(const char *group, const char *format, ...) MBED_PRINTF_METHOD(2, 3);

  int vlprintf(const char *group, const char *format, va_list ap) MBED_PRINTF_METHOD(2, 0);

  // GPSログの書き込み
  void gpsLog(GPSLogData *data);

  // 位置制御情報の書き込み
  void runningLog(RunningData *data);
};
