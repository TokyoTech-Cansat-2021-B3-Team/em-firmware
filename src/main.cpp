#include "mbed.h"
#include "PinAssignment.h"

BufferedSerial serial(USBTX, USBRX);//デバッグ用
I2C i2c(D4,D5);//f303k8デバッグ用


// main() runs in its own thread in the OS
int main() {
    
}