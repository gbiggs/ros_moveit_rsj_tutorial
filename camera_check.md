---
title: の動作確認
date: 2017-05-26
---

既存のROSパッケージを使用してカメラの動作を確認する。

<font color="Black">【準備】</font>

(1) ROSパッケージ『usb_cam』をコンパイルする。

  ```bash
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/bosch-ros-pkg/usb_cam.git
  $ cd ..
  $ catkin_make
  ```

(2) パッケージ『v4l-utils』をインストールする。

  ```bash
  $ sudo apt-get install v4l-utils
  ```

<font color="Black">【実行】</font>

(3) カメラのデバイス番号を確認する。

  ```bash
  $ ls /dev/video*
  ```

(4) 接続中のカメラが対応している解像度を確認する。カメラのデバイス番号が0の場合の例を示す。

  ```bash
  $ v4l2-ctl -d 0 --list-formats-ext
  ```

(5) 必要に応じて、カメラのパラメーターを設定する。

  ```bash
  $ rosparam set usb_cam/pixel_format yuyv #デフォルトのmjpegからyuyvへ変更する。
  ```

(6) roscoreとusb_camを実行する。

  ```bash
  #一つ目のターミナル
  $ roscore
  ```
  ```bash
  #二つ目のターミナル
  $ rosrun usb_cam usb_cam_node
  ```

(7) どのようなROSトピックが流れているかを確認する。

  ```bash
  #三つ目のターミナル
  $ rostopic list　#/usb_cam/image_rawが存在することを確認する。
  ```

(8) 画像を表示する。

  ```bash
  #三つ目のターミナル
  $ rosrun image_view image_view image:=/usb_cam/image_raw
  ```

　次のようなユーザーインターフェースが表示されたら、正しく動作している。また、このユーザーインターフェースのボタンを利用することで、画像を保存することができる。

![usb_cam](images/usb_cam.png)



