#include "Variometer.h"

Variometer::Variometer(BME280 *bme280)
    : _thread(),            //
      _bme280(bme280),      //
      _previousAltitude(0), //
      _currentAltitude(0),  //
      _verticalSpeed(0),    //
      _smaBuffer(),         //
      _smaPtr(0)            //
{}

double Variometer::altitude() {
  double P = _bme280->getPressure();

  return T0 * (pow(P / P0, -(VARIOMETER_R * VARIOMETER_L) / (VARIOMETER_g0 * VARIOMETER_M)) - 1) / VARIOMETER_L;
}

double Variometer::verticalSpeed() {
  return (_currentAltitude - _previousAltitude) /
         (chrono::duration_cast<chrono::microseconds>(VARIOMETER_PERIOD).count() * 0.001 * 0.001);
}

double Variometer::sma(double alt) {
  _smaBuffer[_smaPtr] = alt;
  _smaPtr = (_smaPtr + 1) % VARIOMETER_SMA_N;

  double ret = 0.0;

  for (int i = 0; i < VARIOMETER_SMA_N; i++) {
    ret += _smaBuffer[i];
  }

  ret /= VARIOMETER_SMA_N;

  return ret;
}

void Variometer::threadLoop() {
  while (true) {

    _previousAltitude = _currentAltitude;

    _currentAltitude = sma(altitude());

    _verticalSpeed = verticalSpeed();

    ThisThread::sleep_for(VARIOMETER_PERIOD);
  }
}

void Variometer::start() {
  _bme280->start();

  // 初期値を同期で設定
  _currentAltitude = altitude();
  _previousAltitude = _currentAltitude;
  for (int i = 0; i < VARIOMETER_SMA_N; i++) {
    _smaBuffer[i] = _currentAltitude;
  }

  _thread = make_unique<Thread>(VARIOMETER_THREAD_PRIORITY,   //
                                VARIOMETER_THREAD_STACK_SIZE, //
                                nullptr,                      //
                                VARIOMETER_THREAD_NAME);
  _thread->start(callback(this, &Variometer::threadLoop));
}

void Variometer::stop() {
  _bme280->stop();

  _thread->terminate();
  _thread.reset();
}

double Variometer::getAltitude() const {
  return _currentAltitude;
}

double Variometer::getVerticalSpeed() const {
  return _verticalSpeed;
}
