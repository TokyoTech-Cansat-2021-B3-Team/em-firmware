#include "stabilize.h"
#include "lsm9ds1.h"
#include "WheelControl.h"
#include <math.h>

Stabilize::Stabilize(LSM9DS1* imu, WheelMotor* leftWheelMotor, WheelMotor* rightWheelMotor):
_imu(imu),
_leftWheelMotor(leftWheelMotor),
_rightWheelMotor(rightWheelMotor),
_thread()
{

}

void Stabilize::start() {
    _thread = make_unique<Thread>(STABILIZE_THREAD_PRIORITY, STABILIZE_THREAD_STACK_SIZE, nullptr, STABILIZE_THREAD_NAME);
    _thread->start(callback(this, &Stabilize::threadLoop));
}

void Stabilize::stop() {
  _thread->terminate();
  _thread.reset();
}

void Stabilize::threadLoop(){
    while(true){
        _theta = getTheta(_imu->accX(), _imu->accY(), _imu->accZ());
        double diff = _theta;
        _integral += diff * chrono::duration<float>(STABILIZE_PERIOD).count();;
        _output = diff * _pGain + _integral * _iGain;
        if(_output > 0.0){
            _leftWheelMotor->reverse(_output);
            _rightWheelMotor->reverse(_output);
        }else{
            _leftWheelMotor->forward(-1 * _output);
            _rightWheelMotor->forward(-1 * _output);
        }
        ThisThread::sleep_for(STABILIZE_PERIOD);
    }
}

double Stabilize::getTheta(double accX, double accY, double accZ){
    return  -1 * atan2(-accX, accZ); // 走行EMの座標系で調整
}

double Stabilize::currentOutput(){
    return _output;
}

double Stabilize::currentTheta(){
    return _theta;
}