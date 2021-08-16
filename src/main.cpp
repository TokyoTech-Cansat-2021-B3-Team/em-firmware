#include "mbed.h"

#include "PinAssignment.h"

#include "DCMotor.h"
#include "DrillMotor.h"
#include "QEI.h"
#include "WheelMotor.h"

PwmOut motor1In1(M1_IN1);
PwmOut motor1In2(M1_IN2);

PwmOut motor2In1(M2_IN1);
PwmOut motor2In2(M2_IN2);

PwmOut motor3In1(M3_IN1);
PwmOut motor3In2(M3_IN2);

PwmOut motor4In1(M4_IN1);

WheelMotor leftWheelMotor(&motor1In1, &motor1In2);
WheelMotor rightWheelMotor(&motor2In1, &motor2In2);

DCMotor verticalMotor(&motor3In1, &motor3In2);
DrillMotor drillMotor(&motor4In1);
QEI verticalEncoder(ENC3_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

Thread encoderThread(osPriorityAboveNormal, 1024, nullptr, nullptr);

int previousRev;

void encoderThreadLoop() {
  while (true) {
    printf("pulses: %d, rev: %d, rpm: %d\n", verticalEncoder.getPulses(), verticalEncoder.getRevolutions(),
           (verticalEncoder.getRevolutions() - previousRev) * 60 * 10);

    previousRev = verticalEncoder.getRevolutions();

    ThisThread::sleep_for(100ms);
  }
}

// main() runs in its own thread in the OS
int main() {

  encoderThread.start(encoderThreadLoop);

  while (true) {
    // 上昇
    leftWheelMotor.forward(1.0);
    rightWheelMotor.forward(1.0);
    drillMotor.forward(1.0);
    verticalMotor.forward(1.0);

    ThisThread::sleep_for(2s);

    leftWheelMotor.forward(0.5);
    rightWheelMotor.forward(0.5);
    drillMotor.forward(0.5);
    verticalMotor.forward(0.5);

    ThisThread::sleep_for(2s);

    leftWheelMotor.stop();
    rightWheelMotor.stop();
    drillMotor.stop();
    verticalMotor.stop();

    ThisThread::sleep_for(2s);

    // 下降
    leftWheelMotor.reverse(1.0);
    rightWheelMotor.reverse(1.0);
    verticalMotor.reverse(1.0);

    ThisThread::sleep_for(2s);

    leftWheelMotor.reverse(0.5);
    rightWheelMotor.reverse(0.5);
    verticalMotor.reverse(0.5);

    ThisThread::sleep_for(2s);

    leftWheelMotor.stop();
    rightWheelMotor.stop();
    drillMotor.stop();
    verticalMotor.stop();

    ThisThread::sleep_for(2s);
  }
}
