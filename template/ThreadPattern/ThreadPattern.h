#pragma once

#include "mbed.h"

#define THREAD_PATTERN_THREAD_PRIORITY osPriorityNormal
#define THREAD_PATTERN_THREAD_STACK_SIZE 1024
#define THREAD_PATTERN_THREAD_NAME "ThreadPattern"

class ThreadPattern {
private:
  unique_ptr<Thread> _thread;

public:
private:
  void threadLoop();

public:
  explicit ThreadPattern();

  void start();

  void stop();
};
