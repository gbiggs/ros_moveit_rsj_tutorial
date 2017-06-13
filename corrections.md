---
title: 資料の修正
date: 2017-06-13
---

## 画像処理とマニピュレータの組み合わせ

1. 「ワークスペースのセットアップ」で、自分のワークスペースからのパッケージを利用する場合に、「block_finder」のパッケージ名が変更されました。

   修正前
   : ```shell
     $ cd ~/rsj_2017_application_ws/src
     $ cp -r ~/crane_plus_ws/src/pick_and_placer .
     $ cp -r ~/block_finder_ws/src/block_finder .
     ```

   修正後
   : ```shell
     $ cd ~/rsj_2017_application_ws/src
     $ cp -r ~/crane_plus_ws/src/pick_and_placer .
     $ cp -r ~/block_finder_ws/src/rsj_2017_block_finder .  # <- 変更
     ```

   従って、全ページで「block_finder」パッケージ名を「rsj_2017_block_finder」に変更されました。

1. 「ワークスペースのセットアップ」で、本セミナーに用意されたノードの利用する場合の「block_finder」をクローンするところでは、URLが変更されました。

   修正前
   : ```shell
     $ cd ~/rsj_2017_application_ws/src
     $ git clone https://github.com/Suzuki1984/image_processing.git
     ```
     _注意：パッケージ名が自分製パッケージと異なります。下記の説明で`block_finder`と書いているところに`rsj_block_finder`を利用してください。_

   修正後
   : ```shell
     $ cd ~/rsj_2017_application_ws/src
     $ git clone https://github.com/Suzuki1984/rsj_2017_block_finder  # <- 変更
     ```
     _注意：パッケージ名が自分製パッケージと異なります。下記の説明で`block_finder`と書いているところに`rsj_2017_block_finder`を利用してください。_
