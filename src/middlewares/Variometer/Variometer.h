#pragma once

#include "mbed.h"

#include "BME280.h"

#define VARIOMETER_THREAD_PRIORITY osPriorityAboveNormal
#define VARIOMETER_THREAD_STACK_SIZE 1024
#define VARIOMETER_THREAD_NAME "Variometer"

// https://www.enri.go.jp/report/kenichi/pdf/114_1.pdf

// ICAO 標準大気
#define VARIOMETER_P0 (1013.25 * 100) // 地上の気圧 (Pa)
#define VARIOMETER_g0 (9.80665)       // 地上の重力加速度 (m/s^2)
#define VARIOMETER_T0 (288.15)        // 地上の気温 (K)
#define VARIOMETER_L (-0.0065)        // 気温減率 (K/m)
#define VARIOMETER_M (0.0289644)      // 大気のモル質量 (kg/mol)
#define VARIOMETER_R (8.31432)        // 気体定数 (J/K/mol)

#define VARIOMETER_PERIOD BME280_READ_PERIOD

#define VARIOMETER_SMA_N 5 // 移動平均フィルタ

class Variometer {
private:
  unique_ptr<Thread> _thread;
  BME280 *_bme280;

  const double P0 = VARIOMETER_P0;
  const double T0 = VARIOMETER_T0;

  double _previousAltitude;
  double _currentAltitude;
  double _verticalSpeed;

  double _smaBuffer[VARIOMETER_SMA_N];
  size_t _smaPtr;

public:
private:
  void threadLoop();

  double altitude();

  double verticalSpeed();

  double sma(double alt);

public:
  explicit Variometer(BME280 *bme280);

  void start();

  void stop();

  // 高度 (m)
  double getAltitude() const;

  // 垂直速度 (m/s)
  double getVerticalSpeed() const;
};
