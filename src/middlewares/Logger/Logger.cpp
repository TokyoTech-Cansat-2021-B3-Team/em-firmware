#include "Logger.h"

Logger::Logger(BlockDevice *blockDevice, FileSystem *fileSystem)
    : _blockDevice(blockDevice), //
      _fileSystem(fileSystem),   //
      _messageFile()             //
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
  int err = file->open(_fileSystem, path, O_RDWR | O_CREAT);

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

void Logger::dump(shared_ptr<File> file, const char *path) {
  printf("Dumping \"%s\":\n", path);

  char c;
  while (read(file, &c, 1) != 0) {
    printf("%c", c);
  }
}

void Logger::init() {
  // ファイルシステムのマウント
  mount();

  // メッセージログ用のファイル
  _messageFile = open(LOGGER_MESSAGE_FILE_PATH);

  dump(_messageFile, LOGGER_MESSAGE_FILE_PATH);
}

void Logger::deinit() {
  // メッセージログ用のファイル
  close(_messageFile, LOGGER_MESSAGE_FILE_PATH);
}

int Logger::lprintf(const char *format, ...) {
  va_list ap;
  va_start(ap, format);

  int ret = vlprintf(format, ap);

  va_end(ap);

  return ret;
}

int Logger::vlprintf(const char *format, va_list ap) {

  unique_ptr<char[]> buffer = make_unique<char[]>(LOGGER_PRINTF_BUFFER_SIZE);

  int ret = vsnprintf(buffer.get(), LOGGER_PRINTF_BUFFER_SIZE, format, ap);

  if (ret < 0) {
    return ret;
  }

  write(_messageFile, buffer.get(), ret);

  return ret;
}
