#include "stabilize.h"
#include "WheelControl.h"
#include "lsm9ds1.h"
#include <math.h>

Stabilize::Stabilize(LSM9DS1 *imu, WheelMotor *leftWheelMotor, WheelMotor *rightWheelMotor,
                     WheelControl *leftWheelControl, WheelControl *rightWheelControl)
    : _imu(imu), _leftWheelMotor(leftWheelMotor), _rightWheelMotor(rightWheelMotor),
      _leftWheelControl(leftWheelControl), _rightWheelControl(rightWheelControl), _thread() {}

void Stabilize::start() {
  _thread = make_unique<Thread>(STABILIZE_THREAD_PRIORITY, STABILIZE_THREAD_STACK_SIZE, nullptr, STABILIZE_THREAD_NAME);
  _thread->start(callback(this, &Stabilize::threadLoop));
}

void Stabilize::stop() {
  _thread->terminate();
  _thread.reset();
}

void Stabilize::threadLoop() {
  _leftWheelMotor->forward(1.0);
  _rightWheelMotor->forward(1.0);
  ThisThread::sleep_for(500ms);
  _leftWheelMotor->forward(0);
  _rightWheelMotor->forward(0);
  /*
  if (!checkStabilizerOpend()) {
    invokeStabilizer();
    printf("--------- invoke Stabilier -----------\r\n");
    printf("--------- invoke Stabilier -----------\r\n");
    printf("--------- invoke Stabilier -----------\r\n");
    printf("--------- invoke Stabilier -----------\r\n");
  }
  waitingStabilizerOpening();
  _leftWheelMotor->forward(1);
  _rightWheelMotor->forward(1);
  ThisThread::sleep_for(500ms);
  _output = 1.0;
  while (_output > 0) {
    _output -= 0.01;
    _leftWheelMotor->forward(_output);
    _rightWheelMotor->forward(_output);
    ThisThread::sleep_for(50ms);
  }
  //invokeAntiTolque();
*/
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
  _leftWheelMotor->forward(0.6);
  _rightWheelMotor->forward(0.6);
  ThisThread::sleep_for(700ms);
}

void Stabilize::waitingStabilizerOpening() {
  _state = WAITING_STABILIZER_OPENING;
  _leftWheelMotor->forward(0.0);
  _rightWheelMotor->forward(0.0);
  ThisThread::sleep_for(500ms);
}

void Stabilize::invokeAntiTolque() {
  _state = PREPARING_INVOKE_ANTI_TOLQUE;
  _leftWheelMotor->forward(1);
  _rightWheelMotor->forward(1);
  ThisThread::sleep_for(1s);
  _state = INVOKE_ANTI_TOLQUE;
  _leftWheelMotor->reverse(0.6);
  _rightWheelMotor->reverse(0.6);
  ThisThread::sleep_for(100ms);
  _leftWheelMotor->forward(0);
  _rightWheelMotor->forward(0);
}

bool Stabilize::checkStabilizerOpend() {
  _theta = getTheta(_imu->accX(), _imu->accY(), _imu->accZ());
  if (fabs(_theta) > 2.4434) {
    // 140deg * 180 / pi = 2.44346095279
    return false;
  }
  return true;
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
    _leftWheelControl->setDirection(FOWARD);
    _rightWheelControl->setDirection(FOWARD);
  } else if (dir == CCW) {
    _leftWheelControl->setDirection(REVERSE);
    _rightWheelControl->setDirection(REVERSE);
  }
  _leftWheelControl->start();
  _rightWheelControl->start();
  _leftWheelControl->setTargetSpeed(45); //無負荷では45rpmが最大
  _leftWheelControl->setTargetSpeed(45);
  ThisThread::sleep_for(500ms); // 500msあれば最大速度まで達するであろう
  _leftWheelControl->stop();//WheelControlによるPID制御をオフ
  _rightWheelControl->stop();
  if (dir == CW) {
    changeAllWheelOutput(-1.0);
  } else {
    changeAllWheelOutput(1.0);
  }
  ThisThread::sleep_for(200ms);
}