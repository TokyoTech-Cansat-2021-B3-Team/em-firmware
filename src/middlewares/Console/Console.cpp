#include "Console.h"

Console::Console(MU2 *mu2, Logger *logger) : _mu2(mu2), _logger(logger), _isInit(false) {}

void Console::init() {
  if (!_isInit) {
    _mu2->init();

    _logger->init();

    _isInit = true;
  }
}

int Console::lprintf(const char *group, const char *format, ...) {
  va_list ap;
  va_start(ap, format);

  int ret = vlprintf(group, format, ap);

  va_end(ap);

  return ret;
}

int Console::vlprintf(const char *group, const char *format, va_list ap) {
  unique_ptr<char[]> buffer = make_unique<char[]>(CONSOLE_PRINTF_BUFFER_SIZE);

  // タイムスタンプ
  time_t t = time(nullptr);
  tm *lt = localtime(&t);

  size_t size = 0;

  int ret = snprintf(buffer.get(), CONSOLE_PRINTF_BUFFER_SIZE, "[%04d-%02d-%02dT%02d:%02d:%02d][%s]: ", //
                     lt->tm_year + 1900,                                                                //
                     lt->tm_mon + 1,                                                                    //
                     lt->tm_mday,                                                                       //
                     lt->tm_hour,                                                                       //
                     lt->tm_min,                                                                        //
                     lt->tm_sec,                                                                        //
                     group                                                                              //
  );

  if (ret < 0) {
    return ret;
  }

  size += ret;

  ret = vsnprintf(buffer.get() + size, CONSOLE_PRINTF_BUFFER_SIZE - size, format, ap);

  if (ret < 0) {
    return ret;
  }

  size += ret;

  // 改行までとする
  size = 0;
  while (1) {
    if (buffer[size] == '\r' || buffer[size] == '\n') {
      buffer[size] = '\0';
      break;
    }
    size++;
  }

  _mu2->transmit(buffer.get(), size);

  return ret;
}

int Console::log(const char *group, const char *format, ...) {
  va_list ap;
  va_start(ap, format);

  // ダウンリンク
  int ret = vlprintf(group, format, ap);

  if (ret < 0) {
    return ret;
  }

  // ログ書き込み
  ret = _logger->vlprintf(group, format, ap);

  if (ret < 0) {
    return ret;
  }

  va_end(ap);

  return ret;
}
