#pragma once

#include <cstdint>

using NmeaRmcPacket = struct {
  double utc;              // UTC時刻 hhmmss.sss
  char status;             // A: 有効, V: 無効
  double latitude;         // 緯度 ddmm.mmmm
  char nsIndicator;        // N: 北緯, S: 南緯
  double longitude;        // 経度: dddmm.mmmm
  char ewIndicator;        // E: 東経, W: 西経
  float speedOverGround;   // 対地速度 knots
  float courseOverGround;  // 対地コース degrees
  uint32_t date;           // 日付 ddmmyy
  float magneticVariation; // 磁気偏角 degrees
  char variationDirection; // 磁気偏角の向き E: 東, W: 西
  char mode;               // N: データなし, A: 単独測位, D: DGPS, E: Dead Reckoning
};
