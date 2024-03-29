#include "Logger.h"

Logger::Logger(BlockDevice *blockDevice, FileSystem *fileSystem)
    : _blockDevice(blockDevice), //
      _fileSystem(fileSystem),   //
      _messageFile(),            //
      _gpsFile(),                //
      _isInit(false)             //
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

int Logger::getLastID() {
  size_t retId = 0;

  // Display the root directory
  printf("Opening the root directory... ");
  fflush(stdout);

  DIR *d = opendir("/fs/");

  printf("%s\n", (!d ? "Fail :(" : "OK"));
  if (!d) {
    error("error: %s (%d)\n", strerror(errno), -errno);
  }

  printf("root directory:\n");
  while (true) {
    struct dirent *e = readdir(d);
    if (!e) {
      break;
    }

    size_t fId = 0;
    int err = sscanf(e->d_name, LOGGER_MESSAGE_FILE_NAME_FORMAT, &fId);
    printf("    %s, err: %d, fId: %u\n", e->d_name, err, fId);
    if (err == 1 && //
        fId > retId) {
      retId = fId;
    }
  }

  printf("Closing the root directory... ");
  fflush(stdout);

  int err = closedir(d);
  printf("%s\n", (err < 0 ? "Fail :(" : "OK"));
  if (err < 0) {
    error("error: %s (%d)\n", strerror(errno), -errno);
  }

  return retId;
}

void Logger::dumpMessageLog(size_t id) {
  printf("Dumping \"" LOGGER_MESSAGE_FILE_NAME_FORMAT "\":\n", id);

  char c;
  while (read(_messageFile, &c, 1) != 0) {
    printf("%c", c);
  }
}

void Logger::dumpGPSLog(size_t id) {
  printf("Dumping \"" LOGGER_GPS_FILE_NAME_FORMAT "\":\n", id);

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

void Logger::dumpRunningLog(size_t id) {
  printf("Dumping \"" LOGGER_RUNNING_FILE_NAME_FORMAT "\":\n", id);

  RunningData data;
  while (read(_runningFile, &data, sizeof(data)) != 0) {
    printf("%lf, %lf, %lf, %lf, %lf, %lf %lf\n", //
           data.x, data.y, data.theta, data.leftTargetSpeed, data.rightTargetSpeed, data.leftSpeed, data.rightSpeed);
  }
}

void Logger::init() {
  if (!_isInit) {
    // ファイルシステムのマウント
    mount();

    // 既存のファイル番号を判断
    id = getLastID() + 1;

    char filePath[LOGGER_FILE_PATH_BUFFER_SIZE];

    // メッセージログ用のファイル
    snprintf(filePath, LOGGER_FILE_PATH_BUFFER_SIZE, LOGGER_DIRECTORY LOGGER_MESSAGE_FILE_NAME_FORMAT, id);
    _messageFile = open(filePath);

    //   dumpMessageLog();

    // GPSログ用のファイル
    snprintf(filePath, LOGGER_FILE_PATH_BUFFER_SIZE, LOGGER_DIRECTORY LOGGER_GPS_FILE_NAME_FORMAT, id);
    _gpsFile = open(filePath);

    //   dumpGPSLog();

    //走行シーケンスログ用のファイル
    snprintf(filePath, LOGGER_FILE_PATH_BUFFER_SIZE, LOGGER_DIRECTORY LOGGER_RUNNING_FILE_NAME_FORMAT, id);
    _runningFile = open(filePath);

    // dumpRunningLog();

    _isInit = true;
  }
}

void Logger::deinit() {
  if (_isInit) {
    char filePath[LOGGER_FILE_PATH_BUFFER_SIZE];

    // メッセージログ用のファイル
    snprintf(filePath, LOGGER_FILE_PATH_BUFFER_SIZE, LOGGER_DIRECTORY LOGGER_MESSAGE_FILE_NAME_FORMAT, id);
    close(_messageFile, filePath);

    // GPSログ用のファイル
    snprintf(filePath, LOGGER_FILE_PATH_BUFFER_SIZE, LOGGER_DIRECTORY LOGGER_GPS_FILE_NAME_FORMAT, id);
    close(_gpsFile, filePath);

    // 走行シーケンス用のファイル
    snprintf(filePath, LOGGER_FILE_PATH_BUFFER_SIZE, LOGGER_DIRECTORY LOGGER_RUNNING_FILE_NAME_FORMAT, id);
    close(_runningFile, filePath);

    _isInit = false;
  }
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

  int size = 0;

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

  size += ret;

  ret = vsnprintf(buffer.get() + size, LOGGER_PRINTF_BUFFER_SIZE - size, format, ap);

  if (ret < 0) {
    return ret;
  }

  size += ret;

  write(_messageFile, buffer.get(), size);

  return ret;
}

void Logger::gpsLog(GPSLogData *data) {
  write(_gpsFile, data, sizeof(GPSLogData));
}

void Logger::runningLog(RunningData *data) {
  write(_runningFile, data, sizeof(RunningData));
}
