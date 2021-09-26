#include "mbed.h"

#include "PinAssignment.h"

#include "DCMotor.h"
#include "DrillMotor.h"
#include "QEI.h"
#include "Stepper.h"
#include "WheelMotor.h"
#include <cstdio>

PwmOut motor1In1(M1_IN1);
PwmOut motor1In2(M1_IN2);

PwmOut motor2In1(M2_IN1);
PwmOut motor2In2(M2_IN2);

WheelMotor leftWheelMotor(&motor1In1, &motor1In2);
WheelMotor rightWheelMotor(&motor2In1, &motor2In2);

void waitOutputWheelMotor() {
  for (int i = 0; i < 3; i++) {
    printf(".");
    ThisThread::sleep_for(500ms);
  }
  printf("\r\n");
}

// main() runs in its own thread in the OS
int main() {
  while (true) {
    printf("rotate left wheel motor forward");
    leftWheelMotor.forward(0.8);
    waitOutputWheelMotor();
    leftWheelMotor.stop();
    ThisThread::sleep_for(200ms);
    printf("rotate right wheel motor forward");
    rightWheelMotor.forward(0.8);
    waitOutputWheelMotor();
    rightWheelMotor.stop();
    ThisThread::sleep_for(200ms);
    printf("rotate left & right wheel motor forward");
    leftWheelMotor.forward(0.8);
    rightWheelMotor.forward(0.8);
    waitOutputWheelMotor();
    leftWheelMotor.stop();
    rightWheelMotor.stop();
    ThisThread::sleep_for(200ms);
    printf("rotate left wheel motor reverse");
    leftWheelMotor.reverse(0.8);
    waitOutputWheelMotor();
    leftWheelMotor.stop();
    ThisThread::sleep_for(200ms);
    printf("rotate right wheel motor reverse");
    rightWheelMotor.reverse(0.8);
    waitOutputWheelMotor();
    rightWheelMotor.stop();
    ThisThread::sleep_for(200ms);
    printf("rotate left & right wheel motor reverse");
    leftWheelMotor.reverse(0.8);
    rightWheelMotor.reverse(0.8);
    waitOutputWheelMotor();
    leftWheelMotor.stop();
    rightWheelMotor.stop();
    ThisThread::sleep_for(200ms);
    printf("end test\r\n");
    while(true)ThisThread::sleep_for(100ms);
  }
}
