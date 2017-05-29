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

3. カメラのデバイス番号を確認する。

   ```shell
   $ ls /dev/video*
   ```

4. 接続中のカメラが対応している解像度を確認する。カメラのデバイス番号が0の場合の例を示す。

   ```shell
   $ v4l2-ctl -d 0 --list-formats-ext
   ```

5. 必要に応じて、カメラのパラメーターを設定する。

   ```shell
   $ rosparam set usb_cam/pixel_format yuyv #デフォルトのmjpegからyuyvへ変更する。
   ```

6. roscoreとusb_camを実行する。

   ```shell
   # 一つ目のターミナル
   $ roscore
   ```

   ```shell
   # 二つ目のターミナル
   $ rosrun usb_cam usb_cam_node
   ```

7. どのようなROSトピックが流れているかを確認する。

   ```shell
   # 三つ目のターミナル
   $ rostopic list　#/usb_cam/image_rawが存在することを確認する。
   ```

8. 画像を表示する。

   ```shell
   # 三つ目のターミナル
   $ rosrun image_view image_view image:=/usb_cam/image_raw
   ```

次のようなユーザーインターフェースが表示されたら、正しく動作している。また、このユーザーインターフェースのボタンを利用することで、画像を保存することができる。

![usb_cam](images/usb_cam.png)
