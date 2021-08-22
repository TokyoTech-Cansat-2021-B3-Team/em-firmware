#pragma once

#include "mbed.h"

#include "Logger.h"
#include "MU2.h"

#define CONSOLE_PRINTF_BUFFER_SIZE LOGGER_PRINTF_BUFFER_SIZE

class Console {
private:
  MU2 *_mu2;
  Logger *_logger;

public:
private:
public:
  explicit Console(MU2 *mu2, Logger *logger);

  // 初期化
  // 設定値は事前にEEPROMに設定されているが念のため起動時に設定する
  void init();

  // メッセージダウンリンク
  int lprintf(const char *group, const char *format, ...) MBED_PRINTF_METHOD(2, 3);

  int vlprintf(const char *group, const char *format, va_list ap) MBED_PRINTF_METHOD(2, 0);

  // メッセージログ
  int log(const char *group, const char *format, ...) MBED_PRINTF_METHOD(2, 3);
};
