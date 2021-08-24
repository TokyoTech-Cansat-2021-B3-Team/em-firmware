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
    float diff = _targetSpeed - sensorSpeed;
    float deviation = sensorSpeed - _previousSpeed;
    _integral += diff * chrono::duration<float>(period).count();
    _diff += deviation / chrono::duration<float>(period).count();
    _previousSpeed = sensorSpeed;
    
    _output = diff * _pGain + _integral * _iGain + _diff * _dGain;
}

double WheelPID::getOutput(){
    return _output;
}

void WheelPID::resetIntegral(){
    _integral = 0.0;
}