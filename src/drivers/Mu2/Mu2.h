#pragma once

#include "mbed.h"

#define MU2_SERIAL_BAUDRATE 57600 // UARTボーレート

#define MU2_CMD_TERM "\r\n" // コマンド終端
#define MU2_CMD_UI "@UI"    // ユーザID設定
#define MU2_CMD_GI "@GI"    // グループID設定
#define MU2_CMD_EI "@EI"    // 機器ID設定
#define MU2_CMD_DI "@DI"    // 目的局ID指定
#define MU2_CMD_CH "@CH"    // 使用チャンネル設定
#define MU2_CMD_W_OPT "/W"  // EEPROM書き込みオプション
#define MU2_CMD_TR1 "@DT"   // データ送信コマンド1
#define MU2_TR_MAX_SIZE 255 // 最大送信サイズ

#define MU2_UI "0000" // ユーザID: 0000
#define MU2_GI "00"   // グループID: 00
#define MU2_EI "01"   // 機器ID: 01 (地上局02)
#define MU2_DI "00"   // 目的局ID: 00 (同報)
#define MU2_CH "1A"   // 使用チャンネル: 0x1A (429.4875MHz)

#define MU2_CONTROL_WRITE false // 設定をEEPROMに書き込むかどうか

class Mu2 {
private:
  BufferedSerial *_serial;

public:
private:
  void controlCommand(const char *command, const char *value);

public:
  explicit Mu2(BufferedSerial *serial);

  // 初期化
  // 設定値は事前にEEPROMに設定されているが念のため起動時に設定する
  void init();

  // データ送信
  size_t transmit(const char *data, size_t size);
};