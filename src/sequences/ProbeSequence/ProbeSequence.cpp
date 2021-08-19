#include "ProbeSequence.h"
#include <cstdio>

ProbeSequence::ProbeSequence(DrillMotor *drillMotor, DCMotor *verticalMotor, Stepper *loadingMotor,
                             QEI *verticalEncoder)
    : _thread(), _drillMotor(drillMotor), //
      _verticalMotor(verticalMotor),      //
      _loadingMotor(loadingMotor),        //
      _probeNumber(Probe1),               //
      _verticalEncoder(verticalEncoder)   //
{}

void ProbeSequence::threadLoop() {
  // startが呼ばれるとここから始まる
  // ここにシーケンスを書く

  if (_probeNumber == Probe1) {
    set(20, 0.8, 0.3);

    drilling(72.0, 0.125, 0.8, 5, 50, 0.1, 0.1, 1.0, 0.3, 0.3);
  }

  // 2～4本
  else {
    drilling(72.0, 0.125, 0.8, 5, 50, 0.1, 0.1, 1.0, 0.3, 0.3);
  }
}

void ProbeSequence::set(double L, double stroke, double duty) {
  //上下駆動用モーターを回す
  _verticalMotor->reverse(duty);
  //エンコーダーでセンシング
  _verticalEncoder->reset();
  int i = 0;
  int loop = L / stroke * 6 * 249;
  while (i < loop) {
    i = _verticalEncoder->getPulses();
  }
  //モーター停止
  _verticalMotor->stop();
}

//刺しこみ一連の動作
void ProbeSequence::drilling(){
  
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
