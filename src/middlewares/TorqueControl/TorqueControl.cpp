#include "TorqueControl.h"
#include "MotorSpeed.h"
#include "QEI.h"
#include "WheelMotor.h"
#include "WheelPID.h"
#include "mbed.h"
#include <chrono>
#include <memory>

TorqueControl::TorqueControl(MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed, WheelControl *leftWheelControl,
                             WheelControl *rightWheelControl, WheelPID *leftWheelPID, WheelPID *rightWheelPID)
    : _leftMotorSpeed(leftMotorSpeed), _rightMotorSpeed(rightMotorSpeed), _leftWheelControl(leftWheelControl),
      _rightWheelControl(rightWheelControl), _leftWheelPID(leftWheelPID), _rightWheelPID(rightWheelPID), _thread() {}

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
    updateDecreasingSpeedCount();
    updateDeviation();
    updateDeviationFiltered();
    if (_state == GENERALSPEED) {
      if (checkDecreasingSpeed()) {
        double leftslowingDetectSpeed = _slowingDetectSpeedRatio * _leftWheelControl->targetSpeed();
        double rightslowingDetectSpeed = _slowingDetectSpeedRatio * _rightWheelControl->targetSpeed();
        if (leftSpeed < leftslowingDetectSpeed || rightSpeed < rightslowingDetectSpeed) {
          setSlowState();
        }
      }
    } else if (_state == SLOWSPEED) {
      if (checkSufficientSpeedUp()) {
        setGeneralState();
      }
    }
    ThisThread::sleep_for(TORQUECONTROL_PERIOD);
  }
}

bool TorqueControl::checkDecreasingSpeed() {
  if (_deviationFilteredLeft > _slowingDisableDevivationThreshold && _deviationFilteredRight > _slowingDisableDevivationThreshold) {
    return false;
  }
  if (_decreasingSpeedCount > _decreasingSpeedCountThreshold) {
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
    double _leftspeedThreshold = _leftWheelControl->targetSpeed() * _sufficientSpeedRatio;
    double _rightspeedThreshold = _rightWheelControl->targetSpeed() * _sufficientSpeedRatio;
    if (_leftMotorSpeed->currentSpeedRPM() > _leftspeedThreshold &&
        _rightMotorSpeed->currentSpeedRPM() > _rightspeedThreshold) {
      _sufficientSpeedUpCount++;
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

double TorqueControl::cruiseSpeed() {
  return _cruiseSpeed;
}

void TorqueControl::setSlowState() {
  if (_cruiseSpeed > _slowCruiseSpeed) {
    _leftWheelPID->resetIntegral();
    _rightWheelPID->resetIntegral();
  }
  _state = SLOWSPEED;
  _cruiseSpeed = _slowCruiseSpeed;
}

void TorqueControl::setGeneralState() {
  if (_cruiseSpeed > _generalCruiseSpeed) {
    _leftWheelPID->resetIntegral();
    _rightWheelPID->resetIntegral();
  }
  _state = GENERALSPEED;
  _cruiseSpeed = _generalCruiseSpeed;
}

void TorqueControl::updateDecreasingSpeedCount() {
  if (_state == GENERALSPEED) {
    double _leftspeedThreshold = _leftWheelControl->targetSpeed() * _slowingDetectSpeedRatio;
    double _rightspeedThreshold = _rightWheelControl->targetSpeed() * _slowingDetectSpeedRatio;
    if (_leftMotorSpeed->currentSpeedRPM() < _leftspeedThreshold ||
        _rightMotorSpeed->currentSpeedRPM() < _rightspeedThreshold) {
      _decreasingSpeedCount++;
    } else {
      resetDecreasingSpeedCount();
    }
  }
}

void TorqueControl::resetDecreasingSpeedCount() {
  _decreasingSpeedCount = 0;
}

void TorqueControl::updateDeviation() {
  _deviationLeft = _leftSpeedDifference /
                           ((double)chrono::duration_cast<chrono::milliseconds>(TORQUECONTROL_PERIOD).count() / 1000.0);
  _deviationRight = _rightSpeedDifference /
                           ((double)chrono::duration_cast<chrono::milliseconds>(TORQUECONTROL_PERIOD).count() / 1000.0);
}

void TorqueControl::updateDeviationFiltered() {
  _deviationFilteredLeft =
      (1.0 - _deviationLowpassFilteredRatio) * _deviationFilteredLeft + _deviationLowpassFilteredRatio * _deviationLeft;
  _deviationFilteredRight =
      (1.0 - _deviationLowpassFilteredRatio) * _deviationFilteredRight + _deviationLowpassFilteredRatio * _deviationRight;
}
