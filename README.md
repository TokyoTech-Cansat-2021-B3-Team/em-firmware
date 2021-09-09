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

- 初期化されていない可能性があるので、コンストラクタで依存性を参照することは禁止

- 依存性を用いる初期化はinit関数を定義

## Threadの使用

- クラス内でThreadを使用する場合は、別途容易するパターンに従うこと

- Threadは再開できないので、開始時にnew, 停止時にdeleteする必要がある

- リーク防止のためThreadオブジェクトはunique_ptrで管理する

- start、stop関数の実装

- PriorityやStack Sizeに注意する
