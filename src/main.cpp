#include "mbed.h"

#include "PinAssignment.h"

#include "DCMotor.h"
#include "DrillMotor.h"

PwmOut motor3In1(M3_IN1);
PwmOut motor3In2(M3_IN2);
PwmOut motor4In1(M4_IN1);

DCMotor verticalMotor(&motor3In1, &motor3In2);
DrillMotor drillMotor(&motor4In1);

// main() runs in its own thread in the OS
int main() {
  while (true) {
    // ドリル用モータ
    drillMotor.forward(1.0);

    ThisThread::sleep_for(2s);

    drillMotor.forward(0.5);

    ThisThread::sleep_for(2s);

    drillMotor.stop();

    ThisThread::sleep_for(2s);

    // 上下駆動用モータ上昇
    verticalMotor.forward(1.0);

    ThisThread::sleep_for(2s);

    verticalMotor.forward(0.5);

    ThisThread::sleep_for(2s);

    verticalMotor.stop();

    ThisThread::sleep_for(2s);

    // 上下駆動用モータ下降
    verticalMotor.reverse(1.0);

    ThisThread::sleep_for(2s);

    verticalMotor.reverse(0.5);

    ThisThread::sleep_for(2s);

    verticalMotor.stop();

    ThisThread::sleep_for(2s);
  }
}
