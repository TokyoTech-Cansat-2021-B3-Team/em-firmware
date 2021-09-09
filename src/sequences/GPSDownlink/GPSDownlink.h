#pragma once

#include "mbed.h"

#include "Console.h"
#include "PA1010D.h"

#define GPS_DOWNLINK_THREAD_PRIORITY osPriorityBelowNormal1
#define GPS_DOWNLINK_THREAD_STACK_SIZE 2048
#define GPS_DOWNLINK_THREAD_NAME "GPSDownlink"

#define GPS_DOWNLINK_PERIOD 1s

class GPSDownlink {
private:
public:
private:
  unique_ptr<Thread> _thread;
  EventQueue _queue;

  PA1010D *_pa1010d;
  Console *_console;
  Logger *_logger;

  bool _isStart;

public:
private:
  void threadLoop();

  // 定期実行されるダウンリンクの処理
  void downlink();

public:
  explicit GPSDownlink(PA1010D *pa1010d, Console *console, Logger *logger);

  void start();

  void stop();
};
