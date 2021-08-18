#include "ProbeSequence.h"
#include <cstdio>

ProbeSequence::ProbeSequence(DrillMotor *drillMotor, DCMotor *verticalMotor, Stepper *loadingMotor)
    : _thread(), _drillMotor(drillMotor), //
      _verticalMotor(verticalMotor),      //
      _loadingMotor(loadingMotor),        //
      _probeNumber(Probe1)                //
{}

void ProbeSequence::threadLoop() {
  // startが呼ばれるとここから始まる
  // ここにシーケンスを書く
  int i;

  //1本目
  if(i==1){
    set(20);

    drilling();
  }

  //2～4本
  else{
    drilling();
  }
}

//初期位置に移動
void ProbeSequence::set(float L){

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
