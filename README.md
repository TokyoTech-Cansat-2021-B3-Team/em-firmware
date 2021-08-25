# mbed-template

## 使い方

1. Githubのこのリポジトリのトップページから「Use this template」をクリック

2. リポジトリ名を指定して、新しいリポジトリを作成

3. 新しく作成したリポジトリをClone

## 各ファイルの説明

- ".gitignore"：gitで無視されるファイルの定義

    （mbed-os, BUILD, .mbed等はコミットしないこと）

- ".clang-format", ".clang-tidy"：コードの入力規則を定義 

- "mbed_app.json"：mbedの設定

- "mbed-os.lib"：使用するmbed-osのバージョンを指定（v6.13）

- "src"：ソースファイルを入れるフォルダ
   
    - "main.cpp"：main関数