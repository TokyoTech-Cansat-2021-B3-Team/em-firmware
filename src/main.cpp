#include "mbed.h"

#include "PinAssignment.h"

#include "QEI.h"

#include "WheelMotor.h"
#include "WheelPID.h"
#include "WheelControl.h"

#define SPEED_TASK_PERIOD 10ms
#define PRINT_BUFFER_SIZE 128

char printBuffer[PRINT_BUFFER_SIZE];
BufferedSerial serial(USBTX, USBRX, 115200);

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

//WheelControl leftControl(&leftWheelMotor,&leftPID,&leftEncoder, 1000.0);
//WheelControl rightControl(&rightWheelMotor,&rightPID,&rightEncoder, 249.8);
WheelControl leftControl(&motor1In1,&motor1In2,&leftPID,&leftEncoder, 1000.0);
WheelControl rightControl(&motor2In1,&motor2In2,&rightPID,&rightEncoder, 249.8);


void printThreadLoop(){
    while(true){
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "$L:%f R:%f ;\r\n",leftControl.sensorSpeed(), rightControl.sensorSpeed());
        serial.write(printBuffer,strlen(printBuffer));
        ThisThread::sleep_for(100ms);
    }
}

// main() runs in its own thread in the OS
int main() {
  leftControl.setTargetSpeed(20);
  rightControl.setTargetSpeed(20);
  leftControl.start();
  rightControl.start();
  printThread.start(printThreadLoop);
  int i = 0;
  int j = 0;
  double output[3] = {22.0, 18.0, 0.0};
  while (true) {
    i++;
    if(i*100 > 7000){
        i = 0;
        j++;
        leftControl.setTargetSpeed(output[j%3]);
        rightControl.setTargetSpeed(output[j%3]);
    }
    ThisThread::sleep_for(100ms);
  }
}
