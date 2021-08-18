#include "ProbeSequence.h"

ProbeSequence::ProbeSequence(DrillMotor *drillMotor, DCMotor *verticalMotor, Stepper *loadingMotor)
    : _thread(), _drillMotor(drillMotor), //
      _verticalMotor(verticalMotor),      //
      _loadingMotor(loadingMotor),        //
      _probeNumber(Probe1)                //
{}

void ProbeSequence::threadLoop() {
  // startが呼ばれるとここから始まる
  // ここにシーケンスを書く
}

void ProbeSequence::start(ProbeNumber probeNumber) {
  _probeNumber = probeNumber;

  _thread = make_unique<Thread>(THREAD_PATTERN_THREAD_PRIORITY,   //
                                THREAD_PATTERN_THREAD_STACK_SIZE, //
                                nullptr,                          //
                                THREAD_PATTERN_THREAD_NAME);
  _thread->start(callback(this, &ProbeSequence::threadLoop));
}

void ProbeSequence::stop() {
  _thread->terminate();
  _thread.reset();
}
