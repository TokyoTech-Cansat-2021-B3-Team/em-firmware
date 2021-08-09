#include "Mu2.h"

Mu2::Mu2(BufferedSerial *serial) : _serial(serial) {}

void Mu2::controlCommand(const char *command, const char *value) {
  // コマンド
  _serial->write(command, strlen(command));

  // 値
  _serial->write(value, strlen(value));

  // 書き込みオプション
  if (MU2_CONTROL_WRITE) {
    _serial->write(MU2_CMD_W_OPT, strlen(MU2_CMD_W_OPT));
  }

  // ターミネータ
  _serial->write(MU2_CMD_TERM, strlen(MU2_CMD_TERM));
}

void Mu2::init() {
  // ユーザーID設定
  controlCommand(MU2_CMD_UI, MU2_UI);

  // グループID設定
  controlCommand(MU2_CMD_GI, MU2_GI);

  // 機器ID設定
  controlCommand(MU2_CMD_EI, MU2_EI);

  // 目的局ID指定
  controlCommand(MU2_CMD_DI, MU2_DI);

  // 使用チャンネル設定
  controlCommand(MU2_CMD_CH, MU2_CH);
}

size_t Mu2::transmit(const char *data, size_t size) {
  // サイズチェック
  if (size <= 0) {
    return 0;
  }
  size_t correctSize = size > MU2_TR_MAX_SIZE ? MU2_TR_MAX_SIZE : size;

  // Prefix
  _serial->write(MU2_CMD_TR1, strlen(MU2_CMD_TR1));

  // サイズ(Hex)
  char sizeHex[3];
  snprintf(sizeHex, sizeof(sizeHex), "%02X", correctSize);

  _serial->write(sizeHex, strlen(sizeHex));

  // データ(ASCII)
  _serial->write(data, correctSize);

  // ターミネータ
  _serial->write(MU2_CMD_TERM, sizeof(MU2_CMD_TERM));

  return correctSize;
}
