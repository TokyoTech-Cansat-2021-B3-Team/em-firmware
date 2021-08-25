#pragma once

#include "BlockDevice.h"
#include "FileSystem.h"
#include "KVStore.h"
#include "mbed.h"

#define LOGGER_FORCE_REFORMAT false
#define LOGGER_MESSAGE_FILE_PATH "/MessageLog.txt"
#define LOGGER_GPS_FILE_PATH "/GPSLog.bin"

#define LOGGER_PRINTF_BUFFER_SIZE 256

// シーケンス完了KV
#define LOGGER_KEY_COMPLETE_LANDING "CompleteLanding"            // 着地検知シーケンス完了
#define LOGGER_KEY_COMPLETE_FUSING "CompleteFusing"              // 溶断シーケンス完了
#define LOGGER_KEY_COMPLETE_FIRST_PROBE "CompleteFirstProbe"     // 1本目刺し込みシーケンス完了
#define LOGGER_KEY_COMPLETE_FIRST_RUNNING "CompleteFirstRunning" // 1to2本目走行シーケンス完了
#define LOGGER_KEY_COMPLETE_SECOND_PROBE "CompleteSecondProbe"   // 2本目刺し込みシーケンス完了
#define LOGGER_KEY_COMPLETE_FIRST_RUNNING "CompleteFirstRunning" // 2to3本目走行シーケンス完了
#define LOGGER_KEY_COMPLETE_THIRD_PROBE "CompleteThirdProbe"     // 3本目刺し込みシーケンス完了
#define LOGGER_KEY_COMPLETE_FIRST_RUNNING "CompleteFirstRunning" // 3to4本目走行シーケンス完了
#define LOGGER_KEY_COMPLETE_FOURTH_PROBE "CompleteFourthProbe"   // 4本目刺し込みシーケンス完了

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
  BlockDevice *_fsBlockDevice;
  KVStore *_kvStore;

  // メッセージログ用のファイル
  shared_ptr<File> _messageFile;

  // GPSログ用のファイル
  shared_ptr<File> _gpsFile;

  bool _isInit;

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
  explicit Logger(BlockDevice *fsBlockDevice, FileSystem *fileSystem, KVStore *kvStore);

  void init();

  void deinit();

  template <class T>
  int writeStore(const char *key, T value);

  template <class T>
  int readStore(const char *key, T *value);

  // メッセージログの書き込み
  int lprintf(const char *group, const char *format, ...) MBED_PRINTF_METHOD(2, 3);

  int vlprintf(const char *group, const char *format, va_list ap) MBED_PRINTF_METHOD(2, 0);

  // GPSログの書き込み
  void gpsLog(GPSLogData *data);
};

template <typename T>
int Logger::writeStore(const char *key, T value) {
  // フラッシュに書き込み
  int ret = _kvStore->set(key, &value, sizeof(T), 0);

  if (ret != MBED_SUCCESS) {
    printf("failed to write to \"%s\"\n", key);
  } else {
    printf("succeed in writing to \"%s\"\n", key);
  }

  return ret;
}

template <typename T>
int Logger::readStore(const char *key, T *value) {
  KVStore::info_t info;

  int ret = _kvStore->get_info(key, &info);

  if (ret == MBED_ERROR_ITEM_NOT_FOUND || info.size != sizeof(T)) {
    return ret;
  }

  ret = _kvStore->get(key, value, sizeof(T));

  if (ret != MBED_SUCCESS) {
    printf("failed to read from \"%s\"\n", key);
  } else {
    printf("succeed in actual reading from \"%s\"\n", key);
  }

  return ret;
}
