#include "stabilize.h"
#include "WheelControl.h"
#include "lsm9ds1.h"
#include <math.h>

Stabilize::Stabilize(LSM9DS1 *imu, WheelMotor *leftWheelMotor, WheelMotor *rightWheelMotor)
    : _imu(imu), _leftWheelMotor(leftWheelMotor), _rightWheelMotor(rightWheelMotor), _thread() {}

void Stabilize::start() {
  _thread = make_unique<Thread>(STABILIZE_THREAD_PRIORITY, STABILIZE_THREAD_STACK_SIZE, nullptr, STABILIZE_THREAD_NAME);
  _thread->start(callback(this, &Stabilize::threadLoop));
}

void Stabilize::stop() {
  terminate();
  _thread->terminate();
  _thread.reset();
}

void Stabilize::terminate() {
  _leftWheelMotor->forward(0.0);
  _rightWheelMotor->forward(0.0);
}

void Stabilize::threadLoop() {
  if (!checkStabilizerOpend()) {
    invokeStabilizer();
  }
  waitingStabilizerOpening();
  while (true) {
    _theta = getTheta(_imu->accX(), _imu->accY(), _imu->accZ());
    double diff = _theta - _targetTheta;
    _integral += diff * chrono::duration<float>(STABILIZE_PERIOD).count();
    _output = diff * _pGain + _integral * _iGain;
    // if (_output > 0.6) _output = 0.6;
    // if(_output < -0.6)_output = -0.6;
    if (_output > 0.0) {
      _leftWheelMotor->reverse(_output);
      _rightWheelMotor->reverse(_output);
    } else {
      _leftWheelMotor->forward(-1 * _output);
      _rightWheelMotor->forward(-1 * _output);
    }
    if (checkStabilizeComplete()) {
      _state = COMPLETE_STABILIZE;
    }
    ThisThread::sleep_for(STABILIZE_PERIOD);
  }
}

double Stabilize::getTheta(double accX, double accY, double accZ) {
  // return  -1 * atan2(-accX, accZ); // 走行EMの座標系で調整
  return atan2(accZ, -accY); // FMの座標系で調整
}

double Stabilize::currentOutput() {
  return _output;
}

double Stabilize::currentTheta() {
  return _theta;
}

void Stabilize::invokeStabilizer() {
  _state = INVOKE_STABILIZER;
  pulseTorque(CCW);
}

void Stabilize::waitingStabilizerOpening() {
  _state = WAITING_STABILIZER_OPENING;
  _leftWheelMotor->forward(0.0);
  _rightWheelMotor->forward(0.0);
  ThisThread::sleep_for(500ms);
}

bool Stabilize::checkStabilizerOpend() {
  _theta = getTheta(_imu->accX(), _imu->accY(), _imu->accZ());
  if (fabs(_theta) > 2.4434) {
    // 140deg * 180 / pi = 2.44346095279
    return false;
  }
  return true;
}

bool Stabilize::checkStabilizeComplete() {
  _theta = getTheta(_imu->accX(), _imu->accY(), _imu->accZ());
  double diff = _theta - _targetTheta;
  if (fabs(diff) < 0.07) {
    // 4deg * pi /180 = 0.07
    return true;
  }
  return false;
}

void Stabilize::changeAllWheelOutput(double output) {
  if (output >= 0.0) {
    _leftWheelMotor->forward(output);
    _rightWheelMotor->forward(output);
  } else {
    _leftWheelMotor->reverse(output);
    _rightWheelMotor->reverse(output);
  }
}

void Stabilize::pulseTorque(TORQUE_DIRECTION dir) {
  if (dir == CW) {
    changeAllWheelOutput(1.0);
  } else if (dir == CCW) {
    changeAllWheelOutput(-1.0);
  }
  ThisThread::sleep_for(500ms); // 500msあれば最大速度まで達するであろう
  if (dir == CW) {
    changeAllWheelOutput(-1.0);
  } else {
    changeAllWheelOutput(1.0);
  }
  ThisThread::sleep_for(200ms);
}

Stabilize::STABILIZE_STATE Stabilize::state() {
  return _state;
}