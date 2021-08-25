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
    //変位、ストローク、デューティー比

    //ホルダー回転角度、回転速さ(rps)、ストローク、電極接続長さ、刺しこみ長さ
    //デューティー比(接続、ドリル)、デューティー比(接続、上下)、デューティー比(刺しこみ、ドリル)、デューティー比(刺しこみ、上下)
    //デューティー比(初期位置移動、上下)
    drilling(72.0, 60.0, 0.125, 0.8, 15, 30, 0.5, 0.5, 1.0, 0.3, 1.0);
  }

  // 2～4本
  else {
    //ホルダー回転角度、回転速さ(rps)、ストローク、電極接続長さ、刺しこみ長さ
    //デューティー比(接続、ドリル)、デューティー比(接続、上下)、デューティー比(刺しこみ、ドリル)、デューティー比(刺しこみ、上下)
    //デューティー比(初期位置移動、上下)
    drilling(72.0, 60.0, 0.125, 0.8, 15, 30, 0.5, 0.5, 1.0, 0.3, 1.0);
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
void ProbeSequence::drilling(double angle, double angle_0, double speed, double stroke, double dl, double L,
                             double dutyA_connect, double dutyB_connect, double dutyA_drilling, double dutyB_drilling,
                             double duty_back) {

  //～ホルダー回転～
  _loadingMotor->rotate((angle - angle_0), speed);
  ThisThread::sleep_for(2s);
  _loadingMotor->rotate(angle_0, speed);

  //～電極接続～
  s(dutyA_connect, dutyB_connect, stroke, dl);

  //～刺しこみ～
  s(dutyA_drilling, dutyB_drilling, stroke, L);

  //～初期位置に戻す～
  set(dl + L, stroke, duty_back);
}

void ProbeSequence::s(double pwmA, double pwmB, double stroke, double L) {
  //モーター回転
  _drillMotor->forward(pwmA);
  _verticalMotor->forward(pwmB);

  //エンコーダーでセンシング
  _verticalEncoder->reset();
  int i = 0;
  int loop = L / stroke * 6 * 249;
  while (i < loop) {
    i = _verticalEncoder->getPulses();
  }

  //モーター停止
  _verticalMotor->stop();
  _drillMotor->stop();
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
