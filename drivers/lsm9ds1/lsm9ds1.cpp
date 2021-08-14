#include "lsm9ds1.h"
#include "mbed.h"

LSM9DS1::LSM9DS1(I2C* i2c):
_i2c(i2c),
_thread()
{
}

void LSM9DS1::start() {
    make_unique<Thread>(LSM9DS1_THREAD_PRIORITY, LSM9DS1_THREAD_STACK_SIZE, nullptr, LSM9DS1_THREAD_NAME);
    _thread->start(callback(this, &LSM9DS1::threadLoop));
}

void LSM9DS1::stop() {
  _thread->terminate();
  _thread.reset();
}

void LSM9DS1::threadLoop(){
    ThisThread::sleep_for(LSM9DS1_POLLING_PERIOD);
}