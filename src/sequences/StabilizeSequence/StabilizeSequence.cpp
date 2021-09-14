#include "StabilizeSequence.h"
#include "stabilize.h"

StabilizeSequence::StabilizeSequence(Stabilize *stabilize, Console *console, Logger *logger)
    : _stabilize(stabilize), _console(console), _logger(logger),
      _state(UNDEFINED), _timer(), _thread() {}

void StabilizeSequence::start() {
  _timer.start();
  _console->log("stabilize", "init stabilize sequence\n");
  _logger->init();
  _console->init();
  _thread = make_unique<Thread>(STABILIZESEQUENCE_THREAD_PRIORITY, STABILIZESEQUENCE_THREAD_STACK_SIZE, nullptr,
                                STABILIZESEQUENCE_THREAD_NAME);
  _thread->start(callback(this, &StabilizeSequence::threadLoop));
}

void StabilizeSequence::stop() {
  _thread->terminate();
  _thread.reset();
}

void StabilizeSequence::threadLoop() {
  /*
  位置推定->偏差->制御量決定->モータ回転数変更
  のループはNavigateによってすべて行われる
  そのためシーケンスでは
  ・目標位置設定
  ・一定時間後に強制終了
  ・制御完了後の取次およびロギング
  の部分のみ処理を行う.
  */
  if (_state == WAITING) {
    _stabilize->start();
    _previousTime = _timer.elapsed_time();
  } else {
    // WAITING状態でない状態でstartされた場合は待機
    while (_state != WAITING) {
      ThisThread::sleep_for(STABILIZESEQUENCE_PERIOD);
    }
  }
  //stabilizeはエラーを検知できないので、成功判定orタイムアウトまで無限ループする
  while (true) {
    //正常終了の確認
    if (_stabilize->state() == Stabilize::COMPLETE_STABILIZE) {
      _state = COMPLETE;
      break;
    }
    //強制終了の確認
    if ((_timer.elapsed_time() - _previousTime) > STABILIZESEQUENCE_TERMINATE_TIME) {
      _stabilize->terminate();
      _stabilize->stop();

      setStatus(TERMINATE);
      _console->log("stabilize", "terminate\n");
      break;
    }
    ThisThread::sleep_for(STABILIZESEQUENCE_PERIOD);
  }
  //エラー時もしくは完了時はループから抜ける
}

void StabilizeSequence::setStatus(StabilizeSequenceState state) {
  _state = state;
}

StabilizeSequenceState StabilizeSequence::state() {
  return _state;
}