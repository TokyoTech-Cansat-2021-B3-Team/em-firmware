#include "ThreadPattern.h"

ThreadPattern::ThreadPattern() : _thread() {}

void ThreadPattern::threadLoop() {}

void ThreadPattern::start() {
  if (!_thread) {
    _thread = make_unique<Thread>(THREAD_PATTERN_THREAD_PRIORITY,   //
                                  THREAD_PATTERN_THREAD_STACK_SIZE, //
                                  nullptr,                          //
                                  THREAD_PATTERN_THREAD_NAME);
    _thread->start(callback(this, &ThreadPattern::threadLoop));
  }
}

void ThreadPattern::stop() {
  if (_thread) {
    _thread->terminate();
    _thread.reset();
  }
}
