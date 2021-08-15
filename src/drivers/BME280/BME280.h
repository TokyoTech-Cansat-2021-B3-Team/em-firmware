#pragma once

#include "mbed.h"

#define BME280_THREAD_PRIORITY osPriorityNormal
#define BME280_THREAD_STACK_SIZE 1024
#define BME280_THREAD_NAME "BME280"

#define BME280_I2C_ADDR 0x76U
#define BME280_I2C_WRITE_ADDR ((BME280_I2C_ADDR) << 1U)
#define BME280_I2C_READ_ADDR (((BME280_I2C_ADDR) << 1U) | 0x01U)

#define BME280_CALIB00_ADDR 0x88U
#define BME280_CALIB_SEQ1_SIZE 24U // calib00(0x88) ~ 23(0x9F)
#define BME280_CALIB25_ADDR 0xA1U
#define BME280_CALIB_SEQ2_SIZE 1U // calib25(0xA1)
#define BME280_CALIB26_ADDR 0xE1U
#define BME280_CALIB_SEQ3_SIZE 7U // calib26(0xE1) ~ 33(0xE7)

#define BME280_CALIB_UINT8(byte) (static_cast<uint8_t>(byte))
#define BME280_CALIB_INT8(byte) (static_cast<int8_t>(byte))
#define BME280_CALIB_UINT16(msb, lsb) (static_cast<uint16_t>((msb) << 8U) | (lsb))
#define BME280_CALIB_INT16(msb, lsb) (static_cast<int16_t>(BME280_CALIB_UINT16(msb, lsb)))
#define BME280_CALIB_H4(msb, lsb) (static_cast<uint16_t>((msb) << 4U) | static_cast<uint16_t>((lsb)&0x0FU))
#define BME280_CALIB_H5(msb, lsb)                                                                                      \
  (static_cast<uint16_t>(static_cast<uint16_t>((msb) << 4U) | static_cast<uint16_t>((lsb) >> 4U) & 0x0FU))

#define BME280_CONFIG_ADDR 0xF5U
#define BME280_CONFIG(t_sb, filter)                                                                                    \
  (static_cast<uint8_t>(static_cast<uint8_t>((t_sb)&0b111U) << 5U) |                                                   \
   static_cast<uint8_t>(static_cast<uint8_t>((filter)&0b111U) << 2U))

#define BME280_CTRL_MEAS_ADDR 0xF4U
#define BME280_CTRL_MEAS(osrs_t, osrs_p, mode)                                                                         \
  (static_cast<uint8_t>(static_cast<uint8_t>((osrs_t)&0b111U) << 5U) |                                                 \
   static_cast<uint8_t>(static_cast<uint8_t>(static_cast<uint8_t>((osrs_p)&0b111U) << 2U) |                            \
                        static_cast<uint8_t>((mode)&0b11U)))

#define BME280_CTRL_HUM_ADDR 0xF2U
#define BME280_CTRL_HUM(osrs_h) (static_cast<uint8_t>((osrs_h)&0b111U))

#define BME280_PRESS_MSB 0xF7
#define BME280_PRESS_SEQ_SIZE 3
#define BME280_TEMP_MSB 0xFA
#define BME280_TEMP_SEQ_SIZE 3
#define BME280_HUN_LSB 0xFD
#define BME280_HUM_SEQ_SIZE 2
#define BME280_DATA_SEQ_SIZE ((BME280_PRESS_SEQ_SIZE) + (BME280_TEMP_SEQ_SIZE) + (BME280_HUM_SEQ_SIZE))

#define BME280_PRESS_ADC(msb, lsb, xlsb)                                                                               \
  (static_cast<int32_t>(                                                                                               \
      static_cast<uint32_t>(static_cast<uint32_t>((msb) << 12U) | static_cast<uint32_t>((lsb) << 4U)) |                \
      static_cast<uint32_t>((xlsb) >> 4U)))
#define BME280_TEMP_ADC(msb, lsb, xlsb) BME280_PRESS_ADC(msb, lsb, xlsb)
#define BME280_HUM_ADC(msb, lsb) (static_cast<int16_t>(static_cast<uint16_t>((msb) << 8U) | (lsb)))

#define BME280_PRESS_DOUBLE(value) (static_cast<double>(value) / 256.0)
#define BME280_TEMP_DOUBLE(value) (static_cast<double>(value) / 100.0)
#define BME280_HUM_DOUBLE(value) (static_cast<double>(value) / 1024.0)

#define BME280_T_SB 0b000U   // スタンバイ時間 0.5ms
#define BME280_FILTER 0b000U // IIRフィルタ OFF
#define BME280_OSRS_T 0b001U // 温度オーバーサンプリング x1
#define BME280_OSRS_P 0b001U // 気圧オーバーサンプリング x1
#define BME280_OSRS_H 0b001U // 湿度オーバーサンプリング x1
#define BME280_MODE 0b11U    // モード Normal

#define BME280_READ_PERIOD 100ms

class BME280 {
private:
  using TrimParam = struct {
    uint16_t digT1;
    int16_t digT2;
    int16_t digT3;
    uint16_t digP1;
    int16_t digP2;
    int16_t digP3;
    int16_t digP4;
    int16_t digP5;
    int16_t digP6;
    int16_t digP7;
    int16_t digP8;
    int16_t digP9;
    uint8_t digH1;
    int16_t digH2;
    uint8_t digH3;
    int16_t digH4;
    int16_t digH5;
    int8_t digH6;
  };

  using MeasuredValue = struct {};

  I2C *_i2c;
  unique_ptr<Thread> _thread;

  // 補正係数
  TrimParam _trimParam;

  // t_fine
  int32_t _tFine;

  // データ
  double _pressure;    // 気圧 Pa
  double _tempareture; // 温度 DegC
  double _humidity;    // 湿度 %RH

public:
private:
  void threadLoop();

  // レジスタから読み込み
  int readRegister(uint8_t address, uint8_t *dst, size_t size);

  //　レジスタへの書き込み
  int writeRegister(uint8_t address, uint8_t value);

  // 補正係数読み込み
  void readTrim();

  // コンフィグ書き込み
  void writeConfig();

  // 測定値読み込み
  void readData();

  // 気圧補正
  double compensatePressure(int32_t adc) const;

  // 温度補正
  double compensateTemperature(int32_t adc);

  // 湿度補正
  double compensateHumidity(int32_t adc) const;

public:
  explicit BME280(I2C *i2c);

  void start();

  void stop();

  // 気圧(Pa)
  double getPressure() const;

  // 温度(DegC)
  double getTemprature() const;

  // 湿度(%RH)
  double getHumidity() const;
};
