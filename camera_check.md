---
title: カメラの動作確認
date: 2017-05-29
---

既存のROSパッケージを使用してカメラの動作を確認します。

## 準備

1. ROSパッケージ『usb_cam』をコンパイルします。

   ```shell
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/ros-drivers/usb_cam.git
   $ cd ..
   $ catkin_make
   ```

1. パッケージ『v4l-utils』をインストールします。

   ```shell
   $ sudo apt-get install v4l-utils
   ```

## 実行

1. 内蔵カメラの有無を確認します。ノートパソコンなどで内蔵カメラがある場合はデバイス番号が表示されます。

   ```shell
   $ ls /dev/video*
   ```

1. カメラをUSBでパソコンに繋げます。

1. 外付けカメラのデバイス番号を確認します。

   ```shell
   $ ls /dev/video*
   /dev/video0
   ```

   __各自の環境（ハードウェア）によりデバイス番号が変わります。例えば、ノートパソコンなどで内蔵カメラがある場合は/dev/video1などとなります。__

1. 外付けカメラが対応している解像度を確認します。（※デバイス番号が0の場合の例を示します。）

   ```shell
   $ v4l2-ctl -d 0 --list-formats-ext
   ```

1. 外付けカメラが取得している画像を表示します。

   ```shell
   $ roslaunch usb_cam usb_cam-test.launch
   ```

   次のようなユーザーインターフェースが表示されたら、正しく動作しています。また、このユーザーインターフェースのボタンを利用することで、画像を保存することができます。

   ![usb_cam](images/usb_cam.png)

   『Ctrl』キー＋『c』キーで終了します。

## 補足

- 必要に応じて、カメラを切り替えます。

   ```shell
　　　#デフォルトのmjpegからyuyvへ変更する。
   $ rosparam set usb_cam/video_device "/dev/video1"
   ```

- 必要に応じて、カメラのパラメーターを設定します。

   ```shell
　　　#デフォルトのmjpegからyuyvへ変更する。
   $ rosparam set usb_cam/pixel_format yuyv
   ```

以上



