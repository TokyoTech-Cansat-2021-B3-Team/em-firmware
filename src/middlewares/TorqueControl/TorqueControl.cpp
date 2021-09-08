#include "TorqueControl.h"
#include "MotorSpeed.h"
#include "QEI.h"
#include "WheelMotor.h"
#include "WheelPID.h"
#include "mbed.h"
#include <memory>

TorqueControl::TorqueControl(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed)
    : _leftMotorSpeed(leftMotorSpeed), _rightMotorSpeed(rightMotorSpeed), _thread() {}

void TorqueControl::start() {
  _thread = make_unique<Thread>(TORQUECONTROL_THREAD_PRIORITY, TORQUECONTROL_THREAD_STACK_SIZE, nullptr,
                                TORQUECONTROL_THREAD_NAME);
  _thread->start(callback(this, &TorqueControl::threadLoop));
}

void TorqueControl::stop() {
  _thread->terminate();
  _thread.reset();
}

void TorqueControl::threadLoop() {
  while (true) {
    double leftSpeed = _leftMotorSpeed->currentSpeedRPM();
    double rightSpeed = _rightMotorSpeed->currentSpeedRPM();
    _leftSpeedDifference = leftSpeed - _leftPreviousSpeed;
    _rightSpeedDifference = rightSpeed - _rightPreviousSpeed;
    _leftPreviousSpeed = leftSpeed;
    _rightPreviousSpeed = rightSpeed;
    updateSufficientSpeedUpCount();
    if (_state == GENERALSPEED) {
      if (checkDecreasingSpeed()) {
        _state = SLOWSPEED;
        resetSufficientSpeedUpCount();
      }
    } else if (_state == SLOWSPEED) {
      if (checkSufficientSpeedUp()) {
        _state = GENERALSPEED;
      }
    }
    ThisThread::sleep_for(TORQUECONTROL_PERIOD);
  }
}

bool TorqueControl::checkDecreasingSpeed() {
  if (_leftSpeedDifference < 0 || _rightSpeedDifference < 0) {
    return true;
  }
  return false;
}

void TorqueControl::setGeneralCruiseSpeed(double speed) {
  _generalCruiseSpeed = speed;
}

void TorqueControl::setSlowCruiseSpeed(double speed) {
  _slowCruiseSpeed = speed;
}

bool TorqueControl::checkSufficientSpeedUp() {
  if (_sufficientSpeedUpCount > _sufficientSpeedUpCountThreshold) {
    return true;
  }
  return false;
}

void TorqueControl::updateSufficientSpeedUpCount() {
  if (_state == SLOWSPEED) {
    double _speedThreshold = _slowCruiseSpeed * _sufficientSpeedUpCount;
    if (_leftMotorSpeed->currentSpeedRPM() > _speedThreshold && _rightMotorSpeed->currentSpeedRPM() > _speedThreshold) {
      _sufficientSpeedUpCount++;
      // SLOWSPEEDでは左右のモータに回転数差を設けないことを要求する仕様になっている
      // SLOWSPEEDにおいて左右のモータに回転数差を設けてしまう場合にはこの実装は破綻する恐れが高い
      // navigation側でSLOWSPEEDにおいては位置制御をおこなわない実装を追加する
    } else {
      resetSufficientSpeedUpCount();
    }
  }
}

void TorqueControl::resetSufficientSpeedUpCount() {
  _sufficientSpeedUpCount = 0;
}

bool TorqueControl::checkNavigatable() {
  if (_state == GENERALSPEED) {
    return true;
  }
  return false;
}