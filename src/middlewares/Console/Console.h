#pragma once

#include "mbed.h"

#include "MU2.h"

#define CONSOLE_PRINTF_BUFFER_SIZE 256

class Console {
private:
  MU2 *_mu2;

public:
private:
public:
  explicit Console(MU2 *mu2);

  // 初期化
  // 設定値は事前にEEPROMに設定されているが念のため起動時に設定する
  void init();

  // メッセージログの送信
  int lprintf(const char *format, ...) MBED_PRINTF_METHOD(1, 2);

  int vlprintf(const char *format, va_list ap) MBED_PRINTF_METHOD(1, 0);
};
