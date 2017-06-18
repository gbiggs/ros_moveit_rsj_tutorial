---
title: 資料の修正
date: 2017-06-15
---

## 画像処理とOpenCVの利用

1. 「画像処理パッケージの設定」でソースのURLが修正されました。

   修正前
   : ```shell
     $ cd ~/block_finder_ws/src
     $ git clone git@github.com:Suzuki1984/rsj_2017_block_finder.git
     $ ls
     CMakeLists.txt  rsj_2017_block_finder  usb_cam
     ```

   修正後
   : ```shell
     $ cd ~/block_finder_ws/src
     $ git clone https://github.com/Suzuki1984/rsj_2017_block_finder.git
     $ ls
     CMakeLists.txt  rsj_2017_block_finder  usb_cam
     ```

## ROSの便利機能

1. シミュレータ上でCRANE+を利用するために新しく必要になったパッケージがあります。シミュレータを起動する前に下記でこのパッケージをインストールしてください。

   ```shell
   $ sudo apt-get install ros-kinetic-joint-trajectory-controllers
   $ sudo apt-get install ros-kinetic-effort-controllers
   ```

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

1. 「ワークスペースのセットアップ」で、本セミナーに用意されたノードの利用する場合の「pick_and_placer」をクローンするところでは、`cd` の先が修正されました。

   修正前
   : ```shell
     $ cd ~/rsj_2017_application_ws/src
     $ git clone https://github.com/gbiggs/rsj_2017_pick_and_placer.git
     $ cd ~/rsj_2017_pick_and_placer
     $ git checkout full_application_version
     ```

   修正後
   : ```shell
     $ cd ~/rsj_2017_application_ws/src
     $ git clone https://github.com/gbiggs/rsj_2017_pick_and_placer.git
     $ cd rsj_2017_pick_and_placer           # <- 変更
     $ git checkout full_application_version
     ```

1. 「ワークスペースのセットアップ」で、本セミナーに用意されたノードの利用する場合の「block_finder」をクローンするところでは、URLが変更されました。

   修正前
   : ```shell
     $ cd ~/rsj_2017_application_ws/src
     $ git clone https://github.com/Suzuki1984/image_processing.git
     ```

   修正後
   : ```shell
     $ cd ~/rsj_2017_application_ws/src
     $ git clone https://github.com/Suzuki1984/rsj_2017_block_finder  # <- 変更
     ```

1. 「ワークスペースのセットアップ」で、本セミナーに利用するカメラノードのリポジトリが移動されました。

   修正前
   : 他の必要なノードもワークスペースに入れます.

     ```shell
     $ cd ~/rsj_2017_application_ws/src
     $ git clone https://github.com/gbiggs/crane_plus_arm.git
     $ git clone https://github.com/ros-drivers/usb_cam.git
     ```

   修正後
   : 他の必要なノードもワークスペースに入れます.

     ```shell
     $ cd ~/rsj_2017_application_ws/src
     $ git clone https://github.com/gbiggs/crane_plus_arm.git
     $ git clone https://github.com/ros-drivers/usb_cam.git  # <- 変更
     ```

1. 「カメラ姿勢のカリブレーション」で、カメラの起動する方法が変更されました。

   修正前
   : カメラも起動することが必要です。新しい端末で下記を実行します。

     ```shell
     $ cd ~/rsj_2017_application_ws
     $ source devel/setup.bash
     $ roslaunch rsj_2017_block_finder start_camera.launch
     ```

   修正後
   : カメラも起動することが必要です。新しい端末で下記を実行します。

     ```shell
     $ cd ~/rsj_2017_application_ws
     $ source devel/setup.bash
     $ rosrun usb_cam usb_cam_node __name:=camera _camera_name:="elecom_ucam" \
         _camera_frame_id:="camera_link" _video_device:="/dev/video0" _image_width:=640 \
         _image_height480 _pixel_format:=yuyv _io_method:=mmap
     ```

1. 「カメラ姿勢のカリブレーション」で、`world`座標系と`base_link`座標系をつなげるためのコマンドが抜けました。

   修正前
   : カメラは`world`座標系に対してカリブレーションします。でも、`crane_plus_hardware`の`start_arm_standalone.launch`はマニピュレータを`base_link`座標系に置きます。一時的に`world`と`base_link`の関係を示すことが必要です。新しい端末で下記を実行すると、`world`と`base_link`の差を`tf`に送信します。（ゼロにしたので、`world`と`base_link`の中央点は一緒だと示しています。）

     ```shell
     $ rosrun
     ```

   修正前
   : カメラは`world`座標系に対してカリブレーションします。でも、`crane_plus_hardware`の`start_arm_standalone.launch`はマニピュレータを`base_link`座標系に置きます。一時的に`world`と`base_link`の関係を示すことが必要です。新しい端末で下記を実行すると、`world`と`base_link`の差を`tf`に送信します。（ゼロにしたので、`world`と`base_link`の中央点は一緒だと示しています。）

     ```shell
     $ rosrun tf static_transform_publisher 0 0 0 0 0 0 world base_link 10
     ```

1. 「カメラ姿勢のカリブレーション」で、カメラカリブレーションのlaunchを「同じ端末」と書いていることは間違いです。別の端末で行います。

   修正前
   : そして、カリブレーションプロセス自体を始めます。上記と同じ端末で下記を実行します。


   修正前
   : そして、カリブレーションプロセス自体を始めます。

1. 「ハードウェア構成を定義」で、カメラの姿勢をする行は３５行目です。

   修正前
   : 計算したカメラの姿勢を上記のファイルに定義します。27行目にある`<origin xyz="0 0 0" rpy="0 0 0"/>`を編集して、カメラカリブレーションが出力した位置と角度を入力します。


   修正前
   : 計算したカメラの姿勢を上記のファイルに定義します。35行目にある`<origin xyz="0 0 0" rpy="0 0 0"/>`を編集して、カメラカリブレーションが出力した位置と角度を入力します。
