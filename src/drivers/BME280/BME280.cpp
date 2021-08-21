#include "BME280.h"

BME280::BME280(I2C *i2c)
    : _i2c(i2c),         //
      _thread(),         //
      _trimParam(),      //
      _tFine(0),         //
      _pressure(0.0),    //
      _tempareture(0.0), //
      _humidity(0.0)     //
{}

void BME280::start() {
  _thread = make_unique<Thread>(BME280_THREAD_PRIORITY,   //
                                BME280_THREAD_STACK_SIZE, //
                                nullptr,                  //
                                BME280_THREAD_NAME);
  _thread->start(callback(this, &BME280::threadLoop));
}

void BME280::stop() {
  _thread->terminate();
  _thread.reset();
}

void BME280::threadLoop() {
  readTrim();

  writeConfig();

  while (true) {
    readData();

    ThisThread::sleep_for(BME280_READ_PERIOD);
  }
}

int BME280::readRegister(uint8_t address, uint8_t *dst, size_t size) {
  int ret = 0; // 0: ACK, nonzero: NACK

  ret = _i2c->write(BME280_I2C_WRITE_ADDR, reinterpret_cast<const char *>(&address), 1, true);

  if (ret != 0) {
    return ret;
  }

  ret = _i2c->read(BME280_I2C_READ_ADDR, reinterpret_cast<char *>(dst), size);

  return ret;
}

int BME280::writeRegister(uint8_t address, uint8_t value) {
  uint8_t writeBuffer[2] = {address, value};

  return _i2c->write(BME280_I2C_WRITE_ADDR, reinterpret_cast<const char *>(writeBuffer), sizeof(writeBuffer));
}

void BME280::readTrim() {
  uint8_t calibSeq1[BME280_CALIB_SEQ1_SIZE];
  uint8_t calibSeq2[BME280_CALIB_SEQ2_SIZE];
  uint8_t calibSeq3[BME280_CALIB_SEQ3_SIZE];

  if (readRegister(BME280_CALIB00_ADDR, calibSeq1, BME280_CALIB_SEQ1_SIZE) != 0 ||
      readRegister(BME280_CALIB25_ADDR, calibSeq2, BME280_CALIB_SEQ2_SIZE) != 0 ||
      readRegister(BME280_CALIB26_ADDR, calibSeq3, BME280_CALIB_SEQ3_SIZE) != 0) {
    // 補正係数読み込み失敗
  }

  _trimParam.digT1 = BME280_CALIB_UINT16(calibSeq1[1], calibSeq1[0]);
  _trimParam.digT2 = BME280_CALIB_INT16(calibSeq1[3], calibSeq1[2]);
  _trimParam.digT3 = BME280_CALIB_INT16(calibSeq1[5], calibSeq1[4]);
  _trimParam.digP1 = BME280_CALIB_UINT16(calibSeq1[7], calibSeq1[6]);
  _trimParam.digP2 = BME280_CALIB_INT16(calibSeq1[9], calibSeq1[8]);
  _trimParam.digP3 = BME280_CALIB_INT16(calibSeq1[11], calibSeq1[10]);
  _trimParam.digP4 = BME280_CALIB_INT16(calibSeq1[13], calibSeq1[12]);
  _trimParam.digP5 = BME280_CALIB_INT16(calibSeq1[15], calibSeq1[14]);
  _trimParam.digP6 = BME280_CALIB_INT16(calibSeq1[17], calibSeq1[16]);
  _trimParam.digP7 = BME280_CALIB_INT16(calibSeq1[19], calibSeq1[18]);
  _trimParam.digP8 = BME280_CALIB_INT16(calibSeq1[21], calibSeq1[20]);
  _trimParam.digP9 = BME280_CALIB_INT16(calibSeq1[23], calibSeq1[22]);
  _trimParam.digH1 = BME280_CALIB_UINT8(calibSeq2[0]);
  _trimParam.digH2 = BME280_CALIB_INT16(calibSeq3[1], calibSeq3[0]);
  _trimParam.digH3 = BME280_CALIB_UINT8(calibSeq3[2]);
  _trimParam.digH4 = BME280_CALIB_H4(calibSeq3[3], calibSeq3[4]);
  _trimParam.digH5 = BME280_CALIB_H5(calibSeq3[5], calibSeq3[4]);
  _trimParam.digH6 = BME280_CALIB_INT8(calibSeq3[6]);
}

void BME280::writeConfig() {
  // 順番が重要
  if (writeRegister(BME280_CTRL_HUM_ADDR, BME280_CTRL_HUM(BME280_OSRS_H)) != 0 ||
      writeRegister(BME280_CTRL_MEAS_ADDR, BME280_CTRL_MEAS(BME280_OSRS_T, BME280_OSRS_P, BME280_MODE)) != 0 ||
      writeRegister(BME280_CONFIG_ADDR, BME280_CONFIG(BME280_T_SB, BME280_FILTER)) != 0) {
    // コンフィグ書き込み失敗
  }
}

void BME280::readData() {
  uint8_t readBuffer[BME280_DATA_SEQ_SIZE];

  if (readRegister(BME280_PRESS_MSB, readBuffer, BME280_DATA_SEQ_SIZE) != 0) {
    // 測定値読み込み失敗
  }

  int32_t pressureADC = BME280_PRESS_ADC(readBuffer[0], readBuffer[1], readBuffer[2]);
  int32_t temparetureADC = BME280_TEMP_ADC(readBuffer[3], readBuffer[4], readBuffer[5]);
  int32_t humidityADC = BME280_HUM_ADC(readBuffer[6], readBuffer[7]);

  _pressure = compensatePressure(pressureADC);
  _tempareture = compensateTemperature(temparetureADC);
  _humidity = compensateHumidity(humidityADC);
}

double BME280::compensatePressure(int32_t adc) const {
  int64_t var1, var2, p;
  var1 = ((int64_t)_tFine) - 128000;
  var2 = var1 * var1 * (int64_t)_trimParam.digP6;
  var2 = var2 + ((var1 * (int64_t)_trimParam.digP5) << 17);
  var2 = var2 + (((int64_t)_trimParam.digP4) << 35);
  var1 = ((var1 * var1 * (int64_t)_trimParam.digP3) >> 8) + ((var1 * (int64_t)_trimParam.digP2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_trimParam.digP1) >> 33;
  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_trimParam.digP9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_trimParam.digP8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)_trimParam.digP7) << 4);
  return BME280_PRESS_DOUBLE((uint32_t)p);
}

double BME280::compensateTemperature(int32_t adc) {
  int32_t var1, var2, T;
  var1 = ((((adc >> 3) - ((int32_t)_trimParam.digT1 << 1))) * ((int32_t)_trimParam.digT2)) >> 11;
  var2 = (((((adc >> 4) - ((int32_t)_trimParam.digT1)) * ((adc >> 4) - ((int32_t)_trimParam.digT1))) >> 12) *
          ((int32_t)_trimParam.digT3)) >>
         14;
  _tFine = var1 + var2;
  T = (_tFine * 5 + 128) >> 8;
  return BME280_TEMP_DOUBLE(T);
}

double BME280::compensateHumidity(int32_t adc) const {
  int32_t v_x1_u32r;

  v_x1_u32r = (_tFine - ((int32_t)76800));
  v_x1_u32r = (((((adc << 14) - (((int32_t)_trimParam.digH4) << 20) - (((int32_t)_trimParam.digH5) * v_x1_u32r)) +
                 ((int32_t)16384)) >>
                15) *
               (((((((v_x1_u32r * ((int32_t)_trimParam.digH6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)_trimParam.digH3)) >> 11) + ((int32_t)32768))) >>
                   10) +
                  ((int32_t)2097152)) *
                     ((int32_t)_trimParam.digH2) +
                 8192) >>
                14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)_trimParam.digH1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return BME280_HUM_DOUBLE((uint32_t)(v_x1_u32r >> 12));
}

double BME280::getPressure() const {
  return _pressure;
}

double BME280::getTemprature() const {
  return _tempareture;
}

double BME280::getHumidity() const {
  return _humidity;
}
