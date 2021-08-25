#pragma once

#include "mbed.h"

#include "Console.h"
#include "Fusing.h"

#define FUSING_SEQUENCE_THREAD_PRIORITY osPriorityNormal
#define FUSING_SEQUENCE_THREAD_STACK_SIZE 2048
#define FUSING_SEQUENCE_THREAD_NAME "FusingSequences"

#define FUSING_SEQUENCE_HEAT_DURATION 10s // 加熱時間

#define FUSING_SEQUENCE_TIMEOUT 1min // シーケンスのタイムアウト時間

class FusingSequence {
private:
public:
  using FusingSequenceState = enum {
    Running,
    Complete,
  };

private:
  unique_ptr<Thread> _thread;
  Fusing *_fusing;
  Console *_console;

  FusingSequenceState _state;

  bool _isStart;

public:
private:
  void threadLoop();

public:
  explicit FusingSequence(Fusing *fusing, Console *console);

  void start();

  void stop();

  FusingSequenceState state();
};
