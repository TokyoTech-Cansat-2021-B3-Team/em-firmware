#pragma once

#include "mbed.h"
#include <cstdint>

#define LSM9DS1_IMU_I2C_8BIT_ADDR (0x6B << 1)
#define LSM9DS1_MAG_I2C_8BIT_ADDR (0x1E << 1)
#define LSM9DS1_IMU_WHOAMI_ADDR 0x0f
#define LSM9DS1_MAG_WHOAMI_ADDR 0x0f
#define LSM9DS1_IMU_WHOAMI 0b1101000
#define LSM9DS1_MAG_WHOAMI 0b00111101


#define LSM9DS1_THREAD_PRIORITY osPriorityHigh
#define LSM9DS1_THREAD_STACK_SIZE 1024
#define LSM9DS1_THREAD_NAME "LSM9DS1"


#define LSM9DS1_POLLING_PERIOD 2ms
#define LSM9DS1_READ_SIZE 255

#define LSM9DS1_STATUS_UNCONNECTED 0
#define LSM9DS1_STATUS_ERROR 2
#define LSM9DS1_STATUS_FAILED_TO_CONNECT 3
#define LSM9DS1_STATUS_SUCCESS_TO_CONNECT 1

class LSM9DS1{
public:
    explicit LSM9DS1(I2C* i2c);
    void start();
    void stop();
    int getStatus();
    float accX();
    float accY();
    float accZ();
private:
    void threadLoop();
    int startLSM9DS1();
    char i2c_read_memory(const char slave_addr, const char mem_addr);
    void i2c_write_memory(const char slave_addr, const char mem_addr, const char cnt);
    void i2c_read_memories(const char slave_addr, const char mem_addr, char* buf, int length);
    void get_acc_range();
    void get_gyr_range();
    void get_mag_range();
    void get_temp();
    void get_acc();
    void get_gyr();
    void get_mag();
    I2C* _i2c;
    unique_ptr<Thread> _thread;
    float _accMax;
    float _gyrMax;
    float _magMax;
    double _acc[3] = {0.0f, 0.0f, 0.0f};
    double _gyr[3] = {0.0f,0.0f,0.0f};
    double _mag[3] = {0.0f,0.0f,0.0f};
    uint8_t _status = LSM9DS1_STATUS_UNCONNECTED;
};