#include "mbed.h"

#include "PinAssignment.h"

#include "QEI.h"

#include "WheelMotor.h"
#include "MotorSpeed.h"

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

MotorSpeed leftMotorSpeed(&leftEncoder, 1000.0);
MotorSpeed rightMotorSpeed(&rightEncoder, 249.8);

Thread speedThread(osPriorityAboveNormal, 1024, nullptr, nullptr);
Thread printThread(osPriorityAboveNormal, 1024, nullptr, nullptr);

void printThreadLoop(){
    while(true){
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "$L:%f R:%f ;\r\n",leftMotorSpeed.currentSpeedRPM(),rightMotorSpeed.currentSpeedRPM());
        serial.write(printBuffer,strlen(printBuffer));
        ThisThread::sleep_for(100ms);
    }
}

// main() runs in its own thread in the OS
int main() {
    int i = 0;
    int j = 0;
  printThread.start(printThreadLoop);
  leftMotorSpeed.start();
  rightMotorSpeed.start();
  double output[3] = {0.4, 0.2, 0.0};
  while (true) {
    i++;
    if(i*100 > 5000){
        i = 0;
        j++;
        leftWheelMotor.forward(output[j%3]);
        rightWheelMotor.forward(output[j%3]);
    }
    ThisThread::sleep_for(100ms);
  }
}
