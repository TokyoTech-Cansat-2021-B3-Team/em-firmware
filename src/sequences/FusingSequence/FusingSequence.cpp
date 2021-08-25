#include "FusingSequence.h"
#include <chrono>

FusingSequence::FusingSequence(Fusing *fusing, Console *console)
    : _thread(),         //
      _fusing(fusing),   //
      _console(console), //
      _isStart(false)    //
{}

void FusingSequence::threadLoop() {
  // シーケンス開始
  _state = Running;
  _console->log("Fusing", "start Fusing Sequence\n");

  _console->log("Fusing", "start heating %llu seconds\n",
                chrono::duration_cast<chrono::seconds>(FUSING_SEQUENCE_HEAT_DURATION).count());

  _fusing->heat(FUSING_SEQUENCE_HEAT_DURATION);

  _console->log("Fusing", "stop heating\n");

  // シーケンス終了
  _state = Complete;
  _console->log("Fusing", "Fusing Sequence complete\n");
}

void FusingSequence::start() {
  _console->init();

  if (!_isStart) {
    _thread = make_unique<Thread>(FUSING_SEQUENCE_THREAD_PRIORITY,   //
                                  FUSING_SEQUENCE_THREAD_STACK_SIZE, //
                                  nullptr,                           //
                                  FUSING_SEQUENCE_THREAD_NAME);
    _thread->start(callback(this, &FusingSequence::threadLoop));

    _isStart = true;
  }
}

void FusingSequence::stop() {
  if (_isStart) {
    _thread->terminate();
    _thread.reset();

    _isStart = false;
  }
}

FusingSequence::FusingSequenceState FusingSequence::state() {
  return _state;
}
