#include "GPSDownlink.h"

GPSDownlink::GPSDownlink(PA1010D *pa1010d, Console *console, Logger *logger)
    : _thread(),         //
      _queue(),          //
      _pa1010d(pa1010d), //
      _console(console), //
      _logger(logger),   //
      _isStart(false)    //
{
  // イベント登録
  _queue.call_every(GPS_DOWNLINK_PERIOD, callback(this, &GPSDownlink::downlink));
}

void GPSDownlink::threadLoop() {
  // シーケンス開始
  _console->log("GPS", "start GPS Downlink\n");

  _queue.dispatch_forever();
}

void GPSDownlink::downlink() {
  PA1010D::GGAPacket gga;
  _pa1010d->getGGA(&gga);

  // ダウンリンク
  _console->log("GPS", "lat: %c%lf, lng: %c%lf, fix: %u\n", //
                gga.nsIndicator ? gga.nsIndicator : ' ',    //
                gga.latitude,                               //
                gga.nsIndicator ? gga.nsIndicator : ' ',    //
                gga.longitude,                              //
                gga.positionFixIndicator);

  // ロギング
  Logger::GPSLogData gpsLogData = {
      gga.utc,                 //
      gga.latitude,            //
      gga.nsIndicator,         //
      gga.longitude,           //
      gga.ewIndicator,         //
      gga.positionFixIndicator //
  };

  _logger->gpsLog(&gpsLogData);
}

void GPSDownlink::start() {
  _pa1010d->start();
  //   _logger->init();
  _console->init();

  if (!_isStart) {
    _thread = make_unique<Thread>(GPS_DOWNLINK_THREAD_PRIORITY,   //
                                  GPS_DOWNLINK_THREAD_STACK_SIZE, //
                                  nullptr,                        //
                                  GPS_DOWNLINK_THREAD_NAME);
    _thread->start(callback(this, &GPSDownlink::threadLoop));

    _isStart = true;
  }
}

void GPSDownlink::stop() {
  if (_isStart) {
    _thread->terminate();
    _thread.reset();

    _isStart = false;
  }
}
