#pragma once

#include "mbed.h"

#define LSM9DS1_IMU_I2C_ADDR 0x6B
#define LSM9DS1_MAG_I2C_ADDR 0x1E
#define LSM9DS1_IMU_I2C_ADDR_WRITE ((LSM9DS1_IMU_I2C_ADDR) << 1)
#define LSM9DS1_MAG_I2C_ADDR_WRITE ((LSM9DS1_MAG_I2C_ADDR) << 1)
#define LSM9DS1_IMU_I2C_ADDR_READ ((LSM9DS1_IMU_I2C_ADDR) << 1 | 0x01) 
#define LSM9DS1_MAG_I2C_ADDR_READ ((LSM9DS1_MAG_I2C_ADDR) << 1 | 0x01)

#define LSM9DS1_THREAD_PRIORITY osPriorityHigh
#define LSM9DS1_THREAD_STACK_SIZE 1024
#define LSM9DS1_THREAD_NAME "LSM9DS1"

#define LSM9DS1_BUFFER_SIZE 256

#define LSM9DS1_POLLING_PERIOD 2ms
#define LSM9DS1_READ_SIZE 255

class LSM9DS1{
public:
    explicit LSM9DS1(I2C* i2c);
    void start();
    void stop();
private:
    void threadLoop();
    I2C* _i2c;
    unique_ptr<Thread> _thread;
    double acc[3] = {0.0f, 0.0f, 0.0f};
    double gyr[3] = {0.0f,0.0f,0.0f};
    double mag[3] = {0.0f,0.0f,0.0f};
};