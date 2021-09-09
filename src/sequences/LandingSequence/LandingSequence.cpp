#include "LandingSequence.h"

LandingSequence::LandingSequence(Variometer *variometer, Console *console)
    : _thread(),               //
      _variometer(variometer), //
      _console(console),       //
      _isStart(false)          //
{}

void LandingSequence::threadLoop() {
  // シーケンス開始
  _state = Running;
  _console->log("Landing", "start Landing Sequence\n");

  // タイムアウト設定
  Timeout timeout;
  timeout.attach(callback(this, &LandingSequence::timeoutCallback), LANDING_SEQUENCE_TIMEOUT);

  // 落下開始まで待機
  _console->log("Landing", "waiting for start falling\n");

  _state = WaitFalling;
  waitFalling();

  // 落下検知
  _console->log("Landing", "detect falling\n");

  // 着地まで待機
  _console->log("Landing", "waiting for start landing\n");

  _state = WaitLanding;
  waitLanding();

  // 着地検知
  _console->log("Landing", "detect landing\n");

  // シーケンス終了
  _state = Complete;
  _console->log("Landing", "Landing Sequence complete\n");
}

bool LandingSequence::isFalling() {
  return _variometer->getVerticalSpeed() <= LANDING_SEQUENCE_FALLING_THRESHOLD;
}

void LandingSequence::waitFalling() {
  Timer timer;
  timer.start(); // 時間計測

  while (true) {
    // 静止している場合、計測をリセット
    if (!isFalling()) {
      timer.reset();
    }

    // 静止が一定時間継続した場合、着地検出
    if (timer.elapsed_time() > LANDING_SEQUENCE_FALLING_DURATION) {
      break;
    }

    ThisThread::sleep_for(LANDING_SEQUENCE_POLLING_PERIOD);
  }

  timer.stop();
}

void LandingSequence::waitLanding() {
  Timer timer;
  timer.start(); // 時間計測

  while (true) {
    // 落下している場合、計測をリセット
    if (isFalling()) {
      timer.reset();
    }

    // 静止が一定時間継続した場合、着地検出
    if (timer.elapsed_time() > LANDING_SEQUENCE_LANDING_DURATION) {
      break;
    }

    ThisThread::sleep_for(LANDING_SEQUENCE_POLLING_PERIOD);
  }

  timer.stop();
}

void LandingSequence::timeoutCallback() {
  // タイムアウトにより、シーケンス終了
  // stopはISRではじかれるのでmainから行う
  _state = SequenceTimeout;
}

void LandingSequence::start() {
  _console->init();
  _variometer->start();

  if (!_isStart) {
    _thread = make_unique<Thread>(LANDING_SEQUENCE_THREAD_PRIORITY,   //
                                  LANDING_SEQUENCE_THREAD_STACK_SIZE, //
                                  nullptr,                            //
                                  LANDING_SEQUENCE_THREAD_NAME);
    _thread->start(callback(this, &LandingSequence::threadLoop));

    _isStart = true;
  }
}

void LandingSequence::stop() {
  if (_isStart) {
    _thread->terminate();
    _thread.reset();

    _isStart = false;
  }
}

LandingSequence::LandingSequenceState LandingSequence::state() {
  return _state;
}