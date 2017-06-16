---
title: カメラの動作確認
date: 2017-06-16
---

既存のROSパッケージを使用してカメラの動作を確認します。

## 準備

1. 最初に新しいワークスペースを作成します。

   ```shell
   $ mkdir -p ~/block_finder_ws/src/
   $ cd ~/block_finder_ws/src/
   $ catkin_init_workspace
   $ ls
   CMakeLists.txt
   ```

1. ROSパッケージ『usb_cam』をコンパイルします。

   ```shell
   $ cd ~/block_finder_ws/src
   $ git clone https://github.com/ros-drivers/usb_cam.git
   $ cd ~/block_finder_ws
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

1. 外付けカメラをUSBでパソコンに繋げます。

1. 外付けカメラのデバイス番号を確認します。

   ```shell
   $ ls /dev/video*
   /dev/video0
   ```

   __各自の環境（ハードウェア）によりデバイス番号が変わります。例えば、ノートパソコンなどで内蔵カメラがある場合は/dev/video1などとなります。__

1. 外付けカメラが対応している解像度などを確認します。（※デバイス番号が0の場合の例を示します。デバイス番号が0以外の場合は、オプション『d』の値を変更してください。）

   ```shell
   $ v4l2-ctl -d 0 --list-formats-ext
   ```

1. 外付けカメラが取得している画像を表示します。

   1. デバイス番号が0の場合

      ```shell
      $ roslaunch usb_cam usb_cam-test.launch
      ```

   1. デバイス番号が0以外の場合

      ```shell
      $ cd ~/block_finder_ws/src/usb_cam/launch
      $ cp usb_cam-test.launch usb_cam-test_rsj.launch
      $ gedit usb_cam-test_rsj.launch
      # video_deviceを/dev/video1などに変更し、保存する。
      $ roslaunch usb_cam usb_cam-test.launch
      ```

1. 次のようなユーザーインターフェースが表示されたら、正しく動作しています。また、このユーザーインターフェースのボタンを利用することで、画像を保存することができます。

   ![usb_cam](images/usb_cam.png)

1. 『Ctrl』キー＋『c』キーで終了します。

以上



