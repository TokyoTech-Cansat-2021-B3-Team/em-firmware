#include "wheelPID.h"
#include <memory>
#include "mbed.h"

WheelPID::WheelPID()
{
}

void WheelPID::setTargetSpeed(double speed){
    _targetSpeed = speed;
}

void WheelPID::updatePIDOutput(double sensorSpeed, chrono::microseconds period){
    float deviation = _targetSpeed - sensorSpeed;
    _integral += deviation * chrono::duration<float>(period).count();
    _diff += deviation * chrono::duration<float>(period).count();
    _previousSpeed = _sensorSpeed;

    _output = deviation * _pGain + _integral * _iGain + _diff * _dGain;
}

double WheelPID::getOutput(){
    return _output;
}