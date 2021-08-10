#pragma once

#include "mbed.h"

#define PA1010D_I2C_ADDR 0x10
#define PA1010D_I2C_WRITE_ADDR ((PA1010D_I2C_ADDR) << 1)
#define PA1010D_I2C_READ_ADDR ((PA1010D_I2C_ADDR) << 1 | 0x01)

#define PA1010D_THREAD_PRIORITY osPriorityHigh
#define PA1010D_THREAD_STACK_SIZE 1024
#define PA1010D_THREAD_NAME "PA1010D"

#define PA1010D_BUFFER_SIZE 256

#define PA1010D_POLLING_PERIOD 100ms
#define PA1010D_READ_SIZE 255

class PA1010D {
private:
  I2C *_i2c;
  unique_ptr<Thread> _thread;

  char buffer[PA1010D_BUFFER_SIZE];
  size_t bufferPosition;

public:
private:
  void threadLoop();

public:
  explicit PA1010D(I2C *i2c);

  void start();

  void stop();
};
