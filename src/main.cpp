#include "mbed.h"

#include "PinAssignment.h"

#include "QEI.h"

#include "WheelMotor.h"
#include "WheelPID.h"
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

MotorSpeed leftMotorSpeed(&leftEncoder, 298.0);
MotorSpeed rightMotorSpeed(&rightEncoder, 298.0);

Thread speedThread(osPriorityAboveNormal, 1024, nullptr, nullptr);
Thread printThread(osPriorityAboveNormal, 1024, nullptr, nullptr);

WheelPID leftPID;
WheelPID rightPID;

double leftSpeed = 0.0;
double rightSpeed = 0.0;

double pulsesToRpm(int pulses, chrono::microseconds period, double gear_ratio) {
  // エンコーダー：6パルス/回転
  return (double)abs(pulses) * 60.0 / 6.0 / gear_ratio / chrono::duration<double>(period).count(); // RPM
}

void speedThreadLoop() {
    while (true) {
      leftPID.updatePIDOutput(leftMotorSpeed.currentSpeedRPM(), SPEED_TASK_PERIOD);
      rightPID.updatePIDOutput(rightMotorSpeed.currentSpeedRPM(), SPEED_TASK_PERIOD);
      leftWheelMotor.forward(leftPID.getOutput());
      rightWheelMotor.forward(rightPID.getOutput());
      rtos::ThisThread::sleep_for(SPEED_TASK_PERIOD);
  }
}

void printThreadLoop(){
    while(true){
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "$L:%f %f R:%f %f ;\r\n",leftMotorSpeed.currentSpeedRPM(), leftPID.getOutput(), rightMotorSpeed.currentSpeedRPM(), rightPID.getOutput());
        serial.write(printBuffer,strlen(printBuffer));
        ThisThread::sleep_for(100ms);
    }
}

// main() runs in its own thread in the OS
int main() {
  leftMotorSpeed.start();
  rightMotorSpeed.start();
  leftPID.setTargetSpeed(20);
  rightPID.setTargetSpeed(20);
  speedThread.start(speedThreadLoop);
  printThread.start(printThreadLoop);
  int i = 0;
  int j = 0;
  while(true){
    if(i * 100 > 5000){
        double output[3] = {20.0, 10.0, 0.0};
        leftPID.setTargetSpeed(output[j%3]);
        rightPID.setTargetSpeed(output[j%3]);
        if((j%3)==2){
            leftPID.resetIntegral();
            rightPID.resetIntegral();
        }
        j++;
        i = 0;
    }
    ThisThread::sleep_for(100ms);
    i++;
  }
}
