#include "ProbeSequence.h"

ProbeSequence::ProbeSequence(DrillMotor *drillMotor, DCMotor *verticalMotor, Stepper *loadingMotor,
                             QEI *verticalEncoder, Console *console, LSM9DS1 *lsm9ds1)
    : _thread(),                         //
      _drillMotor(drillMotor),           //
      _verticalMotor(verticalMotor),     //
      _loadingMotor(loadingMotor),       //
      _probeNumber(Probe1),              //
      _verticalEncoder(verticalEncoder), //
      _isStart(false),                   //
      _state(Stop),                      //
      _console(console),                 //
      _lsm9ds1(lsm9ds1)                  //
{}

void ProbeSequence::threadLoop() {
  // startが呼ばれるとここから始まる
  // ここにシーケンスを書く

  // シーケンス開始
  _state = Running;
  _console->log("Probe", "Start Probe%d Sequence\n", _probeNumber);

  //   // 1本目のみ初期位置への移動
  //   if (_probeNumber == Probe1) {
  //     _console->log("Probe", "Setup Init Position\n");
  //     verticalMove(PROBE_SEQUENCE_SETUP_VERTICAL_DUTY, PROBE_SEQUENCE_SETUP_LENGTH);
  //   }

  // 1本目は回転角が大きい
  if (_probeNumber == Probe1) {
    // ホルダーの回転
    _console->log("Probe", "Rotate holder to next position\n");
    holderToNext(PROBE_SEQUENCE_HOLDER_ANGLE_FIRST_1, PROBE_SEQUENCE_HOLDER_ANGLE_SECOND);
  } else {
    _console->log("Probe", "Rotate holder to next position\n");
    holderToNext(PROBE_SEQUENCE_HOLDER_ANGLE_FIRST, PROBE_SEQUENCE_HOLDER_ANGLE_SECOND);
  }

  // 電極接続
  _console->log("Probe", "Start Connect to Probe\n");
  connect();

  // 刺し込み
  _console->log("Probe", "Start Drilling\n");
  drilling();

  // 初期位置へ戻る
  _console->log("Probe", "Back to Init Position\n");
  back();

  // 4本目のみ20mm戻す
  if (_probeNumber == Probe4) {
    _console->log("Probe", "return Park Position\n");
    verticalMove(PROBE_SEQUENCE_SETUP_VERTICAL_DUTY, -(PROBE_SEQUENCE_SETUP_LENGTH));
  }

  // シーケンス終了s
  _console->log("Probe", "Probe%d Sequence Complete\n", _probeNumber);
  _state = Complete;
}

void ProbeSequence::holderToNext(double first, double second) {
  // 1段階目
  _loadingMotor->rotate(first, PROBE_SEQUENCE_HOLDER_SPEED);

  // 待ち
  ThisThread::sleep_for(PROBE_SEQUENCE_HOLDER_DELAY);

  _console->log("Probe", "Start Probe Push\n");

  // 電極の押し下げ
  probepush();

  _console->log("Probe", "Complete Probe Push\n");

  // 2段目
  _loadingMotor->rotate(second, PROBE_SEQUENCE_HOLDER_SPEED);
}

void ProbeSequence::probepush() {
  //上下駆動指定量下降
  verticalMove(PROBE_SEQUENCE_PROBEPUSH_VERTICAL_DUTY, //
               PROBE_SEQUENCE_CONNECT_LENGTH + PROBE_SEQUENCE_DRILLING_LENGTH);
  //上下駆動指定量上昇
  verticalMove(PROBE_SEQUENCE_BACK_VERTIVAL_DUTY, //
               -(PROBE_SEQUENCE_CONNECT_LENGTH + PROBE_SEQUENCE_DRILLING_LENGTH));
}

void ProbeSequence::connect() {
  // ドリル回転開始
  _drillMotor->forward(PROBE_SEQUENCE_CONNECT_DRILL_DUTY);

  // 上下駆動指定量下降
  verticalMove(PROBE_SEQUENCE_CONNECT_VERTICAL_DUTY, //
               PROBE_SEQUENCE_CONNECT_LENGTH);

  // ドリル停止
  _drillMotor->stop();
}

void ProbeSequence::drilling() {
  // ドリル回転開始
  _drillMotor->forward(PROBE_SEQUENCE_DRILLING_DRILL_DUTY);

  // 上下駆動指定量下降
  verticalMove(PROBE_SEQUENCE_DRILLING_VERTICAL_DUTY, //
               PROBE_SEQUENCE_DRILLING_LENGTH);

  // ドリル停止
  _drillMotor->stop();
}

void ProbeSequence::back() {
  // 上下駆動
  // 電極接続 + 刺し込み の下降分だけ上昇
  verticalMove(PROBE_SEQUENCE_BACK_VERTIVAL_DUTY, //
               -(PROBE_SEQUENCE_CONNECT_LENGTH + PROBE_SEQUENCE_DRILLING_LENGTH));
}

void ProbeSequence::verticalMove_sSaG(double duty, double L) {

  _verticalEncoder->reset();

  Timer timer;
  timer.start();

  if (L >= 0.0) {
    _verticalMotor->forward(duty);
  } else {
    _verticalMotor->reverse(duty);
  }

  while (revToLength(_verticalEncoder->getRevolutions()) < fabs(L) && //
         timer.elapsed_time() < PROBE_SEQUENCE_VERTICAL_TIMEOUT) {
    printf("actX:%f,actY:%f,actZ:%f,length:%f\n", _lsm9ds1->accX(), _lsm9ds1->accY(), _lsm9ds1->accZ(),
           revToLength(_verticalEncoder->getRevolutions()));
    ThisThread::sleep_for(100ms);
  }

  timer.stop();

  _verticalMotor->stop();
}

void ProbeSequence::verticalMove_rSaG(double duty, double L) {

  _verticalEncoder->reset();

  Timer timer;
  timer.start();

  if (L >= 0.0) {
    _verticalMotor->forward(duty);
  } else {
    _verticalMotor->reverse(duty);
  }

  double GoTime = 5000;   // ms
  double StopTime = 3000; // ms
  int G = 0;
  int S = 0;
  int phase = 0;                                                      // Go:0、Stop:1
  while (revToLength(_verticalEncoder->getRevolutions()) < fabs(L) && //
         timer.elapsed_time() < PROBE_SEQUENCE_VERTICAL_TIMEOUT) {
    printf("actX:%f,actY:%f,actZ:%f,length:%f\n", _lsm9ds1->accX(), _lsm9ds1->accY(), _lsm9ds1->accZ(),
           revToLength(_verticalEncoder->getRevolutions()));
    ThisThread::sleep_for(100ms);

    if (phase == 0) {
      if (G > GoTime / 100) {
        phase = 1;
        G = 0;
        _verticalMotor->stop();
      }
      G += 1;
    } else {
      if (S > StopTime / 100) {
        phase = 0;
        S = 0;
        if (L >= 0.0) {
          _verticalMotor->forward(duty);
        } else {
          _verticalMotor->reverse(duty);
        }
      }
      S += 1;
    }
  }

  timer.stop();

  _verticalMotor->stop();
}

double ProbeSequence::revToLength(int revolution) {
  return ((double)revolution / PROBE_SEQUENCE_VERTICAL_RATIO) * PROBE_SEQUENCE_VERTICAL_LEAD;
}

void ProbeSequence::start(ProbeNumber probeNumber) {
  if (!_isStart) {
    _probeNumber = probeNumber;
    _state = Stop;

    _lsm9ds1->start();

    _thread = make_unique<Thread>(PROBE_SEQUENCE_THREAD_PRIORITY,   //
                                  PROBE_SEQUENCE_THREAD_STACK_SIZE, //
                                  nullptr,                          //
                                  PROBE_SEQUENCE_THREAD_NAME);
    _thread->start(callback(this, &ProbeSequence::threadLoop));

    _isStart = true;
  }
}

void ProbeSequence::stop() {
  if (_isStart) {
    _thread->terminate();
    _thread.reset();

    _isStart = false;
  }
}

ProbeSequence::ProbeSequenceState ProbeSequence::state() {
  return _state;
}
