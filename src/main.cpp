#include "mbed.h"
#include "PinAssignment.h"

#include "lsm9ds1.h"
#include <cstring>

#define PRINT_BUFFER_SIZE 128

BufferedSerial serial(USBTX, USBRX);//デバッグ用
I2C i2c(D4,D5);//f303k8デバッグ用

char printBuffer[PRINT_BUFFER_SIZE];

LSM9DS1 imu(&i2c);

// main() runs in its own thread in the OS
int main() {
    imu.start();
    ThisThread::sleep_for(100ms);
    if(imu.getStatus()==LSM9DS1_STATUS_SUCCESS_TO_CONNECT){
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Succeeded connecting LSM9DS1.\r\n");
        serial.write(printBuffer,strlen(printBuffer));
    }else{
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Failed to connect LSM9DS1.\r\n");
        serial.write(printBuffer,strlen(printBuffer));
    }
    while(true){
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "$%f %f %f;\r\n",imu.accX(), imu.accY(), imu.accZ());
        serial.write(printBuffer,strlen(printBuffer));            
        ThisThread::sleep_for(100ms);
    }
}