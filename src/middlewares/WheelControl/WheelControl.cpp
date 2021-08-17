#include "WheelControl.h"
#include "WheelPID.h"
#include "WheelMotor.h"
#include "QEI.h"
#include <memory>
#include "mbed.h"

WheelControl::WheelControl(WheelMotor* wheelmotor, WheelPID* wheelpid, QEI* encoder, double gyarRatio):
_wheelmotor(wheelmotor),
_wheelpid(wheelpid),
_encoder(encoder),
_gearRatio(gyarRatio),
_direction(FOWARD),
_thread()
{
}

WheelControl::WheelControl(PwmOut* in1, PwmOut* in2, WheelPID* wheelpid, QEI* encoder, double gyarRatio):
_wheelmotor(nullptr),
_in1(in1),
_in2(in2),
_wheelpid(wheelpid),
_encoder(encoder),
_gearRatio(gyarRatio),
_direction(FOWARD),
_thread(){
    in1->period(0.0001);
    in2->period(0.0001);
}

void WheelControl::start() {
    _thread = make_unique<Thread>(WHEELCONTROL_THREAD_PRIORITY, WHEELCONTROL_THREAD_STACK_SIZE, nullptr, WHEELCONTROL_THREAD_NAME);
    _thread->start(callback(this, &WheelControl::threadLoop));
}

void WheelControl::stop() {
  _thread->terminate();
  _thread.reset();
}

void WheelControl::threadLoop(){
    while(true){
        updateSensorSpeed();
        if(_targetSpeed==0.0){
            _output = 0.0;
        }else{
            _wheelpid->updatePIDOutput(_sensorSpeed, WHEELCONTROL_PERIOD);
            _output = _wheelpid->getOutput();
        }
        if(_direction == FOWARD){
            if(_wheelmotor==nullptr){
                *_in1 = _output;
            }else{
                _wheelmotor->forward(_output);
            }
        }
        else if(_direction == REVERSE){
            if(_wheelmotor==nullptr){
            }else{
                _wheelmotor->reverse(_output);
            }
        }
        ThisThread::sleep_for(WHEELCONTROL_PERIOD);
    }
}

void WheelControl::setTargetSpeed(double speed){
    _targetSpeed = speed;
    _wheelpid->setTargetSpeed(_targetSpeed);
    
}

void WheelControl::updateSensorSpeed(){
    _sensorSpeed = pulsesToRpm(_encoder->getPulses());
    _encoder->reset();
}

double WheelControl::pulsesToRpm(int pulses){
    // エンコーダー：6パルス/回転
    return (double)abs(pulses) * 60.0 / 6.0 / _gearRatio / chrono::duration<double>(WHEELCONTROL_PERIOD).count(); // RPM
}

void WheelControl::setDirection(DIRECTION direction){
    _direction = direction;
}

double WheelControl::sensorSpeed(){
    return _sensorSpeed;
}