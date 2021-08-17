#include "mbed.h"

#include "PinAssignment.h"

#include "QEI.h"

#include "WheelMotor.h"
#include "WheelPID.h"

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
      leftSpeed = pulsesToRpm(leftEncoder.getPulses(), SPEED_TASK_PERIOD, 1000.0);
      leftEncoder.reset();
      rightSpeed = pulsesToRpm(rightEncoder.getPulses(), SPEED_TASK_PERIOD, 249.8);
      rightEncoder.reset();
      leftPID.updatePIDOutput(leftSpeed, SPEED_TASK_PERIOD);
      rightPID.updatePIDOutput(rightSpeed, SPEED_TASK_PERIOD);
      //leftWheelMotor.forward(leftPID.getOutput());
      //rightWheelMotor.forward(rightPID.getOutput());
      motor1In1 = leftPID.getOutput();
      motor2In1 = rightPID.getOutput();
      rtos::ThisThread::sleep_for(SPEED_TASK_PERIOD);
  }
}

void printThreadLoop(){
    while(true){
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "$L:%f R:%f ;\r\n",leftSpeed,rightSpeed);
        serial.write(printBuffer,strlen(printBuffer));
        ThisThread::sleep_for(100ms);
    }
}

// main() runs in its own thread in the OS
int main() {
  leftPID.setTargetSpeed(20);
  rightPID.setTargetSpeed(20);
  speedThread.start(speedThreadLoop);
  printThread.start(printThreadLoop);
  int i = 0;
  int j = 0;
  double output[3] = {20.0, 10.0, 0.0};
  while (true) {
    i++;
    if(i*100 > 5000){
        i = 0;
        j++;
        leftPID.setTargetSpeed(output[j%3]);
        rightPID.setTargetSpeed(output[j%3]);
    }
    ThisThread::sleep_for(100ms);
  }
}
