---
title: カメラの動作確認
date: 2017-05-29
---

既存のROSパッケージを使用してカメラの動作を確認する。

## 準備

1. ROSパッケージ『usb_cam』をコンパイルする。

   ```shell
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/bosch-ros-pkg/usb_cam.git
   $ cd ..
   $ catkin_make
   ```

1. パッケージ『v4l-utils』をインストールする。

   ```shell
   $ sudo apt-get install v4l-utils
   ```

## 実行

1. カメラをUSBでパソコンにつながります。

1. カメラのデバイス番号を確認する。

   ```shell
   $ ls /dev/video*
   /dev/video0
   ```

   __ハードウェアによりデバイス番号が変わります。また、ノートパソコンに付属のカメラがあると複数のデバイスが表示される場合もあります。__

1. 接続中のカメラが対応している解像度を確認する。カメラのデバイス番号が0の場合の例を示す。

   ```shell
   $ v4l2-ctl -d 0 --list-formats-ext
   ```

1. 必要に応じて、カメラのパラメーターを設定する。

   ```shell
   $ rosparam set usb_cam/pixel_format yuyv #デフォルトのmjpegからyuyvへ変更する。
   ```

1. roscoreとusb_camを実行する。

   ```shell
   # 一つ目のターミナル
   $ roscore
   ```

   ```shell
   # 二つ目のターミナル
   $ rosrun usb_cam usb_cam_node
   ```

1. どのようなROSトピックが流れているかを確認する。

   ```shell
   # 三つ目のターミナル
   $ rostopic list　#/usb_cam/image_rawが存在することを確認する。
   ```

1. 画像を表示する。

   ```shell
   # 三つ目のターミナル
   $ rosrun image_view image_view image:=/usb_cam/image_raw
   ```

次のようなユーザーインターフェースが表示されたら、正しく動作している。また、このユーザーインターフェースのボタンを利用することで、画像を保存することができる。

![usb_cam](images/usb_cam.png)
