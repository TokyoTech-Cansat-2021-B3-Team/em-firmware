#include "WheelControl.h"

WheelControl::WheelControl(WheelMotor *wheelmotor, WheelPID *wheelpid, MotorSpeed *motorSpeed)
    : _wheelmotor(wheelmotor), _wheelpid(wheelpid), _motorSpeed(motorSpeed), _direction(FOWARD), _thread() {}

void WheelControl::start() {
  _thread = make_unique<Thread>(WHEELCONTROL_THREAD_PRIORITY, WHEELCONTROL_THREAD_STACK_SIZE, nullptr,
                                WHEELCONTROL_THREAD_NAME);
  _thread->start(callback(this, &WheelControl::threadLoop));
}

void WheelControl::stop() {
  _thread->terminate();
  _thread.reset();
}

void WheelControl::threadLoop() {
  while (true) {
    updateSensorSpeed();
    if (_targetSpeed == 0.0) {
      _output = 0.0;
      _wheelpid->resetIntegral();
      _wheelpid->updatePIDOutput(_sensorSpeed, WHEELCONTROL_PERIOD);
    } else {
      _wheelpid->updatePIDOutput(_sensorSpeed, WHEELCONTROL_PERIOD);
      _output = _wheelpid->getOutput();
    }
    if (_direction == FOWARD) {
      if (_wheelmotor == nullptr) {
        *_in1 = _output;
      } else {
        _wheelmotor->forward(_output);
      }
    } else if (_direction == REVERSE) {
      if (_wheelmotor == nullptr) {
      } else {
        _wheelmotor->reverse(_output);
      }
    }
    ThisThread::sleep_for(WHEELCONTROL_PERIOD);
  }
}

void WheelControl::setTargetSpeed(double speed) {
  _targetSpeed = speed;
  _wheelpid->setTargetSpeed(_targetSpeed);
}

void WheelControl::updateSensorSpeed() {
  _sensorSpeed = _motorSpeed->currentSpeedRPM();
}

void WheelControl::setDirection(DIRECTION direction) {
  _direction = direction;
}
