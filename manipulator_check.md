---
title: マニピュレータの動作確認
date: 2017-05-30
---

簡単なプログラムを利用してマニピュレータのサーボの動作を確認します。

## ソフトウェアのコンパイル

1. サーボドライバーソフトウェアをコンパイルします。端末を起動し、以下のコマンドを実行します。

   ```shell
   $ cd ~/Downloads/
   $ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
   $ cd DynamixelSDK/c/build/linux64/
   $ make
   mkdir -p ./.objects/
   gcc -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c -I../../include -m64 -fPIC -g -c
       ../../src/dynamixel_sdk/group_bulk_read.c -o .objects/group_bulk_read.o
   gcc -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c -I../../include -m64 -fPIC -g -c
       ../../src/dynamixel_sdk/group_bulk_write.c -o .objects/group_bulk_write.o
   gcc -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c -I../../include -m64 -fPIC -g -c
       ../../src/dynamixel_sdk/group_sync_read.c -o .objects/group_sync_read.o
   gcc -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c -I../../include -m64 -fPIC -g -c
       ../../src/dynamixel_sdk/group_sync_write.c -o .objects/group_sync_write.o
   gcc -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c -I../../include -m64 -fPIC -g -c
       ../../src/dynamixel_sdk/packet_handler.c -o .objects/packet_handler.o
   gcc -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c -I../../include -m64 -fPIC -g -c
       ../../src/dynamixel_sdk/port_handler.c -o .objects/port_handler.o
   gcc -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c -I../../include -m64 -fPIC -g -c
       ../../src/dynamixel_sdk/protocol1_packet_handler.c -o .objects/protocol1_packet_handler.o
   gcc -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c -I../../include -m64 -fPIC -g -c
       ../../src/dynamixel_sdk/protocol2_packet_handler.c -o .objects/protocol2_packet_handler.o
   gcc -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c -I../../include -m64 -fPIC -g -c
       ../../src/dynamixel_sdk_linux/port_handler_linux.c -o .objects/port_handler_linux.o
   g++ -shared -fPIC -m64 -o ./libdxl_x64_c.so ./.objects/group_bulk_read.o
       ./.objects/group_bulk_write.o ./.objects/group_sync_read.o ./.objects/group_sync_write.o
       ./.objects/packet_handler.o ./.objects/port_handler.o
       ./.objects/protocol1_packet_handler.o ./.objects/protocol2_packet_handler.o
       ./.objects/port_handler_linux.o -lrt
   $
   ```

1. サーボ確認プログラムのソースを取得します。

   ```shell
   $ cd ~/Downloads/
   $ git clone https://github.com/gbiggs/dynamixel_servo_check.git
   $ cd dynamixel_servo_check
   ```

1. サーボドライバーのライブラリとヘッダーをコピーします。

   ```shell
   $ cp -r ~/Downloads/DynamixelSDK/c/include/dynamixel_sdk* .
   $ cp ~/Downloads/DynamixelSDK/c/build/linux64/libdxl_x64_c.so .
   $ ls
   CMakeLists.txt  dynamixel_sdk.h      dynamixel_sdk_windows  LICENSE    servo_check.c
   dynamixel_sdk   dynamixel_sdk_linux  libdxl_x64_c.so        README.md
   ```

1. サーボ確認プログラムをコンパイルします。

   ```shell
   $ mkdir build
   $ cd build
   $ cmake ../
   -- The C compiler identification is GNU 5.4.0
   -- The CXX compiler identification is GNU 5.4.0
   -- Check for working C compiler: /usr/bin/cc
   -- Check for working C compiler: /usr/bin/cc -- works
   -- Detecting C compiler ABI info
   -- Detecting C compiler ABI info - done
   -- Detecting C compile features
   -- Detecting C compile features - done
   -- Check for working CXX compiler: /usr/bin/c++
   -- Check for working CXX compiler: /usr/bin/c++ -- works
   -- Detecting CXX compiler ABI info
   -- Detecting CXX compiler ABI info - done
   -- Detecting CXX compile features
   -- Detecting CXX compile features - done
   -- Configuring done
   -- Generating done
   -- Build files have been written to: /home/geoff/Downloads/dynamixel_servo_check/build
   $ make
   [ 50%] Building C object CMakeFiles/servo_check.dir/servo_check.c.o
   [100%] Linking C executable servo_check
   [100%] Built target servo_check
   $ ls
   CMakeCache.txt  CMakeFiles  cmake_install.cmake  Makefile  servo_check
   ```

## ソフトウェアを実行

1. シリアルポートへアクセスするために、下記のコマンドにパーミッショングループに自分のユーザを追加します。

   ```shell
   $ sudo gpasswd -a ユーザ名 dialout
   ```

   上記で`ユーザ名`を自分のユーザ名に変更します。

   上記のようにパーミッショングループにユーザを追加しないとサーボ制御ソフトウェアはハードウェアへアクセスできなくて、エラーになります。

   また、 __実行後は一旦ログアウトして再ログインしてください。__{:style="color: red"}

1. サーボ確認プログラムを実行しサーボの動きを確認します。

   プログラムにサーボIDを指定します。CRANE+のサーボIDは１~５です。

   __注意：プログラムを実行すると指定したサーボは高速で真ん中の位置に移動します。電源を入れる前にマニピュレータをまっすぐ ___上向きに近い姿勢___ にしてください。__{:style="color: red"}

   ```shell
   $ sudo ./servo_check 1
   Opened port
   Changed buadrate
   Dynamixel has been successfully connected
   [ID:003] GoalPos:512  PresPos:760
   [ID:003] GoalPos:512  PresPos:757
   [ID:003] GoalPos:512  PresPos:744
   [ID:003] GoalPos:512  PresPos:730
   [ID:003] GoalPos:512  PresPos:718
   [ID:003] GoalPos:512  PresPos:705
   [ID:003] GoalPos:512  PresPos:687
   [ID:003] GoalPos:512  PresPos:672
   [ID:003] GoalPos:512  PresPos:656
   [ID:003] GoalPos:512  PresPos:640
   [ID:003] GoalPos:512  PresPos:623
   [ID:003] GoalPos:512  PresPos:604
   [ID:003] GoalPos:512  PresPos:586
   [ID:003] GoalPos:512  PresPos:564
   [ID:003] GoalPos:512  PresPos:543
   [ID:003] GoalPos:512  PresPos:522
   ```

1. １から５まで、前サーボの動作を確認しましょう。
