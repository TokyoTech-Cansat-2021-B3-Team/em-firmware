#include "MotorSpeed.h"

MotorSpeed::MotorSpeed(QEI *encoder, double gyarRatio) : _encoder(encoder), _gyarRatio(gyarRatio), _thread() {}

void MotorSpeed::start() {
  _thread =
      make_unique<Thread>(MOTORSPEED_THREAD_PRIORITY, MOTORSPEED_THREAD_STACK_SIZE, nullptr, MOTORSPEED_THREAD_NAME);
  _thread->start(callback(this, &MotorSpeed::threadLoop));
}

void MotorSpeed::stop() {
  _thread->terminate();
  _thread.reset();
}

void MotorSpeed::threadLoop() {
  while (true) {
    _motorSpeedRPM = pulsesToRpm(_encoder->getPulses(), MOTORSPEED_PERIOD);
    _encoder->reset();
    ThisThread::sleep_for(MOTORSPEED_PERIOD);
  }
}

double MotorSpeed::currentSpeedRPM() {
  return _motorSpeedRPM;
}

double MotorSpeed::currentSpeedRPS() {
  return _motorSpeedRPM * 2.0 * PI / 60.0;
}

double MotorSpeed::pulsesToRpm(int pulses, chrono::microseconds period) {
  // エンコーダー：6パルス/回転
  return (double)abs(pulses) * 60.0 / 6.0 / _gyarRatio / chrono::duration<double>(period).count(); // RPM
}
