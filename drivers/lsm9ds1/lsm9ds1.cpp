#include "lsm9ds1.h"
#include <memory>
#include "mbed.h"

LSM9DS1::LSM9DS1(I2C* i2c):
_i2c(i2c),
_thread()
{
}

void LSM9DS1::start() {
    _thread = make_unique<Thread>(LSM9DS1_THREAD_PRIORITY, LSM9DS1_THREAD_STACK_SIZE, nullptr, LSM9DS1_THREAD_NAME);
    _thread->start(callback(this, &LSM9DS1::threadLoop));
}

void LSM9DS1::stop() {
  _thread->terminate();
  _thread.reset();
}

void LSM9DS1::threadLoop(){
    if(startLSM9DS1()==-1){
        status = LSM9DS1_STATUS_FAILED_TO_CONNECT;  
    }
    status = LSM9DS1_STATUS_SUCCESS_TO_CONNECT;
    while(true){
        ThisThread::sleep_for(LSM9DS1_POLLING_PERIOD);
    }
}

int LSM9DS1::startLSM9DS1(){
    char rst_AG = 0;
    char rst_M = 0;
    //lsm9ds1への接続を確認
    rst_AG = i2c_read_memory(LSM9DS1_IMU_I2C_8BIT_ADDR, LSM9DS1_IMU_WHOAMI_ADDR);
    rst_M  = i2c_read_memory(LSM9DS1_MAG_I2C_8BIT_ADDR, LSM9DS1_MAG_WHOAMI_ADDR);
    if(rst_AG == LSM9DS1_IMU_WHOAMI && rst_M == LSM9DS1_MAG_WHOAMI){
        //succeeded connect AG and M
    }else{
        return -1;
    }
    //ジャイロセンサの設定
    char config = i2c_read_memory(LSM9DS1_IMU_I2C_8BIT_ADDR, 0x10);
    config = config | (0b001 << 5);
    i2c_write_memory(LSM9DS1_IMU_I2C_8BIT_ADDR, 0x10, config);
    config = i2c_read_memory(LSM9DS1_IMU_I2C_8BIT_ADDR, 0x10);
    if(config == 0x20){
        //ジャイロの有効化に成功
    }else{
        return -1;
    }
    //磁気センサの設定
    //TEMP_COMPの設定
    config = i2c_read_memory(LSM9DS1_MAG_I2C_8BIT_ADDR, 0x20);
    config = config | (1 << 7);//CTRL_REG1_M の TEMP_COMP を有効化
    i2c_write_memory(LSM9DS1_MAG_I2C_8BIT_ADDR, 0x20, config);
    config = i2c_read_memory(LSM9DS1_MAG_I2C_8BIT_ADDR, 0x20);
    if(config == 0x98){
        //TEMP_COMPの設定に成功
    }else{
        return -1;
    }
    //磁気センサの有効化
    config = i2c_read_memory(LSM9DS1_MAG_I2C_8BIT_ADDR, 0x22);
    config = config & ~(1); //CTRL_REG3_M の MD0 を 0
    config = config & ~(1<<1); //CTRL_REG3_M の MD1 を 0
    i2c_write_memory(LSM9DS1_MAG_I2C_8BIT_ADDR, 0x22, config);
    config = i2c_read_memory(LSM9DS1_MAG_I2C_8BIT_ADDR, 0x22);
    if(config==0x00){
        //磁気センサの有効化に成功
    }else{
        return -1;
    }
    return 0;
}

int LSM9DS1::getStatus(){
    return status;
}

char LSM9DS1::i2c_read_memory(const char slave_addr, const char mem_addr){
    char buf = -1;
    _i2c->write(slave_addr,&mem_addr,1);
    _i2c->read(slave_addr|1,&buf,1);
    return buf;
}

void LSM9DS1::i2c_write_memory(const char slave_addr, const char mem_addr, const char cnt){
    char tmp[2];
    tmp[0] = mem_addr;
    tmp[1] = cnt;
    _i2c->write(slave_addr,tmp,2);
}

void LSM9DS1::i2c_read_memories(const char slave_addr, const char mem_addr, char* buf, int length){
    _i2c->write(slave_addr, &mem_addr,1);
    _i2c->read(slave_addr | 1, (char*)buf, length, true);
}
