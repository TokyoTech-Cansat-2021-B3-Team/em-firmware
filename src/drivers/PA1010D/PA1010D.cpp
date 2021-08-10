#include "PA1010D.h"
#include <memory>

PA1010D::PA1010D(I2C *i2c) : _i2c(i2c), _thread(), buffer(), bufferPosition(0) {}

void PA1010D::start() {
  _thread = make_unique<Thread>(PA1010D_THREAD_PRIORITY, PA1010D_THREAD_STACK_SIZE, nullptr, PA1010D_THREAD_NAME);
  _thread->start(callback(this, &PA1010D::threadLoop));
}

void PA1010D::stop() {
  _thread->terminate();
  _thread.reset();
}