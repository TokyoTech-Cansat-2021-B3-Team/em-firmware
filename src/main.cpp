#include "mbed.h"
#include "PinAssignment.h"

#include "lsm9ds1.h"
#include <cstring>

#define PRINT_BUFFER_SIZE 128

BufferedSerial serial(USBTX, USBRX);//
I2C i2c(D4,D5);//

char printBuffer[PRINT_BUFFER_SIZE];

LSM9DS1 imu(&i2c);

#include "PinAssignment.h"

#include "QEI.h"

#include "WheelMotor.h"
#include "WheelPID.h"
#include "WheelControl.h"

#define SPEED_TASK_PERIOD 10ms
#define PRINT_BUFFER_SIZE 128

PwmOut motor1In1(M1_IN1);
PwmOut motor1In2(M1_IN2);

PwmOut motor2In1(M2_IN1);
PwmOut motor2In2(M2_IN2);

WheelMotor leftWheelMotor(&motor1In1, &motor1In2);
WheelMotor rightWheelMotor(&motor2In1, &motor2In2);

QEI leftEncoder(ENC1_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);
QEI rightEncoder(ENC2_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

Thread printThread(osPriorityAboveNormal, 1024, nullptr, nullptr);

WheelPID leftPID;
WheelPID rightPID;

WheelControl leftControl(&leftWheelMotor,&leftPID,&leftEncoder, 1000.0);
WheelControl rightControl(&rightWheelMotor,&rightPID,&rightEncoder, 249.8);
//WheelControl leftControl(&motor1In1,&motor1In2,&leftPID,&leftEncoder, 1000.0);
//WheelControl rightControl(&motor2In1,&motor2In2,&rightPID,&rightEncoder, 249.8);


void printThreadLoop(){
    while(true){
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "$L:%f R:%f ;\r\n",leftControl.sensorSpeed(), rightControl.sensorSpeed());
        serial.write(printBuffer,strlen(printBuffer));
        ThisThread::sleep_for(100ms);
    }
}

// main() runs in its own thread in the OS
int main() {
  printThread.start(printThreadLoop);
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
      snprintf(printBuffer, PRINT_BUFFER_SIZE, "$%f %f %f %f %f %f %f %f %f;\r\n",imu.accX(), imu.accY(), imu.accZ(), imu.gyrX(), imu.gyrY(), imu.gyrZ(), imu.magX(), imu.magY(), imu.magZ());
      serial.write(printBuffer,strlen(printBuffer));            
      ThisThread::sleep_for(100ms);
  }
}
