#include "Logger.h"

Logger::Logger(BlockDevice *blockDevice, FileSystem *fileSystem)
    : _blockDevice(blockDevice), //
      _fileSystem(fileSystem),   //
      _messageFile(),            //
      _gpsFile()                 //
{}

void Logger::mount() {
  // ファイルシステムをマウント
  printf("Mounting the filesystem... ");
  fflush(stdout);

  int err = _fileSystem->mount(_blockDevice);

  printf("%s\n", (err ? "Fail :(" : "OK"));

  if (err || LOGGER_FORCE_REFORMAT) {
    // Reformat if we can't mount the filesystem
    printf("formatting... ");
    fflush(stdout);

    err = _fileSystem->reformat(_blockDevice);

    printf("%s\n", (err ? "Fail :(" : "OK"));
  }
}

shared_ptr<File> Logger::open(const char *path) {

  // ファイルを開く
  printf("Opening \"%s\"... ", path);
  fflush(stdout);

  shared_ptr<File> file = make_shared<File>();
  int err = file->open(_fileSystem, path, O_RDWR | O_CREAT | O_APPEND);

  if (err) {
    file.reset();
  }

  printf("%s\n", (err ? "Fail :(" : "OK"));

  return file;
}

void Logger::close(shared_ptr<File> file, const char *path) {
  // ファイルが開かれている
  if (file) {
    // Close the file which also flushes any cached writes
    printf("Closing \"%s\"... ", path);
    fflush(stdout);

    int err = file->close();

    printf("%s\n", (err ? "Fail :(" : "OK"));
  }
}

int Logger::write(shared_ptr<File> file, const void *buffer, size_t size) {
  int ret = 0;

  // ファイルが開かれているか確認
  if (file) {
    ret = file->write(buffer, size);

    if (ret < 0) {
      return ret;
    }

    ret = file->sync(); // ストレージへ書き込み
  }

  return ret;
}

int Logger::read(shared_ptr<File> file, void *buffer, size_t size) {
  int ret = 0;

  // ファイルが開かれているか確認
  if (file) {
    ret = file->read(buffer, size);
  }

  return ret;
}

void Logger::dumpMessageLog() {
  printf("Dumping \"%s\":\n", LOGGER_MESSAGE_FILE_PATH);

  char c;
  while (read(_messageFile, &c, 1) != 0) {
    printf("%c", c);
  }
}

void Logger::dumpGPSLog() {
  printf("Dumping \"%s\":\n", LOGGER_GPS_FILE_PATH);

  GPSLogData data;
  while (read(_gpsFile, &data, sizeof(data)) != 0) {
    printf("%lf, %lf, %c, %lf, %c, %u\n", //
           data.utc,                      //
           data.latitude,                 //
           data.nsIndicator,              //
           data.longitude,                //
           data.ewIndicator,              //
           data.positionFixIndicator      //
    );
  }
}

void Logger::init() {
  // ファイルシステムのマウント
  mount();

  // メッセージログ用のファイル
  _messageFile = open(LOGGER_MESSAGE_FILE_PATH);

  //   dumpMessageLog();

  // GPSログ用のファイル
  _gpsFile = open(LOGGER_GPS_FILE_PATH);

  //   dumpGPSLog();
}

void Logger::deinit() {
  // メッセージログ用のファイル
  close(_messageFile, LOGGER_MESSAGE_FILE_PATH);

  // GPSログ用のファイル
  close(_gpsFile, LOGGER_GPS_FILE_PATH);
}

int Logger::lprintf(const char *group, const char *format, ...) {
  va_list ap;
  va_start(ap, format);

  int ret = vlprintf(group, format, ap);

  va_end(ap);

  return ret;
}

int Logger::vlprintf(const char *group, const char *format, va_list ap) {
  unique_ptr<char[]> buffer = make_unique<char[]>(LOGGER_PRINTF_BUFFER_SIZE);

  // タイムスタンプ
  time_t t = time(nullptr);
  tm *lt = localtime(&t);

  int ret = snprintf(buffer.get(), LOGGER_PRINTF_BUFFER_SIZE, "[%04d-%02d-%02dT%02d:%02d:%02d][%s]: ", //
                     lt->tm_year + 1900,                                                               //
                     lt->tm_mon + 1,                                                                   //
                     lt->tm_mday,                                                                      //
                     lt->tm_hour,                                                                      //
                     lt->tm_min,                                                                       //
                     lt->tm_sec,                                                                       //
                     group                                                                             //
  );

  if (ret < 0) {
    return ret;
  }

  write(_messageFile, buffer.get(), ret);

  ret = vsnprintf(buffer.get(), LOGGER_PRINTF_BUFFER_SIZE, format, ap);

  if (ret < 0) {
    return ret;
  }

  write(_messageFile, buffer.get(), ret);

  return ret;
}

void Logger::gpsLog(GPSLogData *data) {
  write(_gpsFile, data, sizeof(GPSLogData));
}
