# em-firmware

EM用プログラム

# コーディングルール

## 命名規則

- クラス名：Upper camel case (HogeHoge)

- Publicメンバ：Lower camel case (hogeHoge)

- Privateメンバ：_ + Lower camel case (_hogeHoge)

- グローバル変数、ローカル変数、関数：Lower camel case (hogeHoge)

- マクロ：Upper snake case (HOGE_HOGE)

- ファイル名：Upper camel case (HogeHoge)

※一部型番などUpper camelですべて大文字は許容 (HOGEHoge)

## ピンアサイン

"PinAssignment.h"をインクルードして参照

## オブジェクト

- 基本的にmain.cppにグローバル変数として持つ

- 依存性注入を基本とする

- 初期化されていない可能性があるので、コンストラクタで依存関係を参照することは禁止

- 依存関係を用いる初期化はinit関数を定義