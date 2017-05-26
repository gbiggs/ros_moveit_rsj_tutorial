---
title: の動作確認
date: 2017-05-26
---

既存のROSパッケージを使用してカメラの動作を確認します。

<div style="text-align: center;">

<font color="Black">【準備】/font>

</div>

(1) ROSパッケージ『usb_cam』をコンパイルします。

  ```bash
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/bosch-ros-pkg/usb_cam.git
  $ cd ..
  $ catkin_make
  ```

(2) パッケージ『v4l-utils』をインストールします。

  ```bash
  $ sudo apt-get install v4l-utils
  ```


<div style="text-align: center;">

<font color="Black"><【実行】/font>

</div>

(3) 接続中のカメラが対応している解像度を確認します。カメラのデバイス番号が0の場合の例を示します。

  ```bash
  $ ls /dev/video*
  $ v4l2-ctl -d 0 --list-formats-ext
  ```

(4) 必要に応じて、カメラのパラメーターを設定します。

  ```bash
  $ rosparam set usb_cam/pixel_format yuyv #mjpegからyuyvへ変更する。
  ```

(5) roscoreとusb_camを実行します。

  ```bash
  #一つ目のターミナル
  $ roscore
  ```
  ```bash
  #二つ目のターミナル
  $ rosrun usb_cam usb_cam_node
  ```

(6) どのようなROSトピックが流れているかを確認します。

  ```bash
  #三つ目のターミナル
  $ rostopic list　#/usb_cam/image_rawが存在することを確認する。
  ```

(7) 画像を表示します。

  ```bash
  #三つ目のターミナル
  $ rosrun image_view image_view image:=/usb_cam/image_raw
  ```

　次のようなユーザーインターフェースが表示されたら、正しく動作しています。このユーザーインターフェースのボタンを利用することで、画像を保存することができます。

<div style="text-align: center;">

![usb_cam](images/usb_cam.png)

</div>



