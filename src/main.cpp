#include "mbed.h"

#include "PinAssignment.h"

#include "DCMotor.h"
#include "DrillMotor.h"
#include "QEI.h"
#include <cstdio>

PwmOut motor3In1(M3_IN1);
PwmOut motor3In2(M3_IN2);
PwmOut motor4In1(M4_IN1);

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
