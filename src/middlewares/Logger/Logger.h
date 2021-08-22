#pragma once

#include "BlockDevice.h"
#include "FileSystem.h"
#include "mbed.h"

#define LOGGER_FORCE_REFORMAT false
#define LOGGER_MESSAGE_FILE_PATH "/MessageLog.txt"

#define LOGGER_PRINTF_BUFFER_SIZE 256

class Logger {
private:
  FileSystem *_fileSystem;
  BlockDevice *_blockDevice;

  // メッセージログ用のファイル
  shared_ptr<File> _messageFile;


public:
private:
  void mount();

  shared_ptr<File> open(const char *path);

  void close(shared_ptr<File> file, const char *path);

  int write(shared_ptr<File> file, const void *buffer, size_t size);

  int read(shared_ptr<File> file, void *buffer, size_t size);

  void dump(shared_ptr<File> file, const char *path);

public:
  explicit Logger(BlockDevice *blockDevice, FileSystem *fileSystem);

  void init();

  void deinit();

  // メッセージログの書き込み
  int lprintf(const char *format, ...) MBED_PRINTF_METHOD(1, 2);

  int vlprintf(const char *format, va_list ap) MBED_PRINTF_METHOD(1, 0);
};
