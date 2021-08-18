#include "mbed.h"

#include "PinAssignment.h"

#include "DCMotor.h"
#include "DrillMotor.h"
#include "QEI.h"
#include "Stepper.h"
#include "WheelMotor.h"

PwmOut motor1In1(M1_IN1);
PwmOut motor1In2(M1_IN2);

PwmOut motor2In1(M2_IN1);
PwmOut motor2In2(M2_IN2);

PwmOut motor3In1(M3_IN1);
PwmOut motor3In2(M3_IN2);

PwmOut motor4In1(M4_IN1);

DigitalOut motor5Enable(M5_ENABLE);
DigitalOut motor5Step(M5_STEP);

WheelMotor leftWheelMotor(&motor1In1, &motor1In2);
WheelMotor rightWheelMotor(&motor2In1, &motor2In2);

DCMotor verticalMotor(&motor3In1, &motor3In2);
DrillMotor drillMotor(&motor4In1);
QEI verticalEncoder(ENC3_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

Stepper loadingMotor(&motor5Step, &motor5Enable);

Thread encoderThread(osPriorityAboveNormal, 1024, nullptr, "encoder");

Thread stepperThread(osPriorityRealtime, 1024, nullptr, "stepper");

int previousRev;

void encoderThreadLoop() {
  while (true) {
    printf("pulses: %d, rev: %d, rpm: %d\n", verticalEncoder.getPulses(), verticalEncoder.getRevolutions(),
           (verticalEncoder.getRevolutions() - previousRev) * 60 * 10);

    previousRev = verticalEncoder.getRevolutions();

    ThisThread::sleep_for(100ms);
  }
}

void stepperThreadLoop() {
  while (true) {
    loadingMotor.rotate(360.0 / 7.0, 0.3);

    ThisThread::sleep_for(500ms);
  }
}

// main() runs in its own thread in the OS
int main() {
  loadingMotor.idleCurrent(false);

  encoderThread.start(encoderThreadLoop);

  stepperThread.start(stepperThreadLoop);

  while (true) {
    // 上昇
    for (int i = 0; i < 1000; i++) {
      //   printf("%d\n", i);

      leftWheelMotor.forward(i * 0.001);
      rightWheelMotor.forward(i * 0.001);

      verticalMotor.forward(i * 0.001);
      drillMotor.forward(i * 0.001);

      ThisThread::sleep_for(1ms);
    }

    for (int i = 1000; i >= 0; i--) {
      //   printf("%d\n", i);

      leftWheelMotor.forward(i * 0.001);
      rightWheelMotor.forward(i * 0.001);

      verticalMotor.forward(i * 0.001);
      drillMotor.forward(i * 0.001);

      ThisThread::sleep_for(1ms);
    }

    // 下降
    for (int i = 0; i < 1000; i++) {
      //   printf("%d\n", i);

      leftWheelMotor.reverse(i * 0.001);
      rightWheelMotor.reverse(i * 0.001);

      verticalMotor.reverse(i * 0.001);

      ThisThread::sleep_for(1ms);
    }

    for (int i = 1000; i >= 0; i--) {
      //   printf("%d\n", i);

      leftWheelMotor.reverse(i * 0.001);
      rightWheelMotor.reverse(i * 0.001);

      verticalMotor.reverse(i * 0.001);

      ThisThread::sleep_for(1ms);
    }
  }
}
