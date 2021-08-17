#include "wheelPID.h"
#include <memory>
#include "mbed.h"

WheelPID::WheelPID(WheelMotor* wheelmotor, QEI* qei):
_wheelmotor(wheelmotor),
_qei(qei),
_thread()
{
}

void WheelPID::start() {
    _thread = make_unique<Thread>(WHEELPID_THREAD_PRIORITY, WHEELPID_THREAD_STACK_SIZE, nullptr, WHEELPID_THREAD_NAME);
    _thread->start(callback(this, &WheelPID::threadLoop));
}

void WheelPID::stop() {
  _thread->terminate();
  _thread.reset();
}

void WheelPID::threadLoop(){
    while(true){
        updateSensorSpeed();
        if(_targetSpeed==0.0){
            _output = 0.0;
        }else{
            updatePIDOutput();
        }
        if(_direction == CW){
            _wheelmotor->forward(_output);
        }else{
            _wheelmotor->forward(_output);
        }
        ThisThread::sleep_for(WHEELPID_PERIOD);
    }
}

void WheelPID::setTargetSpeed(double speed){
    _targetSpeed = speed;
}

void WheelPID::updatePIDOutput(){
    float deviation = _targetSpeed - _sensorSpeed;
    _integral += deviation * chrono::duration<float>(WHEELPID_PERIOD).count();
    _diff += deviation * chrono::duration<float>(WHEELPID_PERIOD).count();
    _previousSpeed = _sensorSpeed;

    _output = deviation * _pGain + _integral * _iGain + _diff * _dGain;
}

void WheelPID::updateSensorSpeed(){
    _sensorSpeed = pulsesToRpm(_qei->getPulses());
    _qei->reset();
}

double WheelPID::pulsesToRpm(int pulses){
    // エンコーダー：6パルス/回転
    return (double)abs(pulses) * 60.0 / 6.0 / _gearRatio / chrono::duration<double>(WHEELPID_PERIOD).count(); // RPM
}

void WheelPID::setGearRatio(double gearRatio){
    _gearRatio = gearRatio;
}

void WheelPID::setDirection(DIRECTION direction){
    _direction = direction;
}