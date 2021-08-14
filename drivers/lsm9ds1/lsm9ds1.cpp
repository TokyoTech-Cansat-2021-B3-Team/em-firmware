#include "lsm9ds1.h"
#include <memory>
#include "mbed.h"

LSM9DS1::LSM9DS1(I2C* i2c):
_i2c(i2c),
_thread(),
_accMax(0.0f),
_gyrMax(0.0f),
_magMax(0.0f)
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
        _status = LSM9DS1_STATUS_FAILED_TO_CONNECT;  
    }
    _status = LSM9DS1_STATUS_SUCCESS_TO_CONNECT;
    get_acc_range();
    get_gyr_range();
    get_mag_range();
    while(true){
        get_temp();
        get_acc();
        get_gyr();
        get_mag();
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
    return _status;
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


void LSM9DS1::get_acc_range(){
    char config = i2c_read_memory(LSM9DS1_IMU_I2C_8BIT_ADDR,0x20);
    float acc_range = 0;
    if((config & 0b11000) == 0b00){
        _accMax = 2;//G   
    }
}

void LSM9DS1::get_gyr_range(){
    char config = i2c_read_memory(LSM9DS1_IMU_I2C_8BIT_ADDR,0x10);
    float gyr_range = 0;
    if((config & 0b11000) == 0b00){
        _gyrMax = 245;//dps
    }
}

void LSM9DS1::get_mag_range(){
    char config = i2c_read_memory(LSM9DS1_MAG_I2C_8BIT_ADDR,0x21);
    float mag_range = 0;
    if((config & 0b11000) == 0b00){
        _magMax = 4;//gaus
    }
}

void LSM9DS1::get_temp(){
    unsigned char temp[2];
    i2c_read_memories(LSM9DS1_IMU_I2C_8BIT_ADDR, 0x15, (char*)temp, 2);
}

void LSM9DS1::get_acc(){
    unsigned char accRaw[6];
    i2c_read_memories(LSM9DS1_IMU_I2C_8BIT_ADDR, 0x28, (char*)accRaw, 6);
    _acc[0] = _accMax * ((float)((int16_t)((accRaw[0*2+1] << 8) | accRaw[0*2+0])) / 32767.0);
    _acc[1] = _accMax * ((float)((int16_t)((accRaw[1*2+1] << 8) | accRaw[1*2+0])) / 32767.0);
    _acc[2] = _accMax * ((float)((int16_t)((accRaw[2*2+1] << 8) | accRaw[2*2+0])) / 32767.0);
}

void LSM9DS1::get_gyr(){
    unsigned char gyrRaw[6];
    i2c_read_memories(LSM9DS1_IMU_I2C_8BIT_ADDR, 0x18, (char*)gyrRaw, 6);
    _gyr[0] = _gyrMax * ((float)((int16_t)((gyrRaw[0*2+1] << 8) | gyrRaw[0*2+0])) / 32767.0);
    _gyr[1] = _gyrMax * ((float)((int16_t)((gyrRaw[1*2+1] << 8) | gyrRaw[1*2+0])) / 32767.0);
    _gyr[2] = _gyrMax * ((float)((int16_t)((gyrRaw[2*2+1] << 8) | gyrRaw[2*2+0])) / 32767.0);              
}

void LSM9DS1::get_mag(){
    unsigned char magRaw[6];
    i2c_read_memories(LSM9DS1_MAG_I2C_8BIT_ADDR, 0x28, (char*)magRaw, 6);
    _mag[0] = _magMax * ((float)((int16_t)((magRaw[0*2+1] << 8) | magRaw[0*2+0])) / 32767.0);
    _mag[1] = _magMax * ((float)((int16_t)((magRaw[1*2+1] << 8) | magRaw[1*2+0])) / 32767.0);
    _mag[2] = _magMax * ((float)((int16_t)((magRaw[2*2+1] << 8) | magRaw[2*2+0])) / 32767.0);     
}

float LSM9DS1::accX(){
    return _acc[0];
}

float LSM9DS1::accY(){
    return _acc[1];
}

float LSM9DS1::accZ(){
    return _acc[2];
}