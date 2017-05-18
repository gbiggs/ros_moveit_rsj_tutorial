---
title: マニピュレータの制御とMoveIt!の利用
date: 2017-05-09
---

マニピュレータ制御には、前セクションで行ったようにサーボごと制御することはもちろん可能です。しかし、マニピュレータのグリッパーの位置を制御したり、スムーズなトラジェクトリーに沿って動かしたりしたいでしょう。ならば、サーボごとのようなローレベル制御は足りません。

ROSでは、マニピュレータを一体型なロボットとして制御しタスクを行うために、[MoveIt!というライブラリ](http://moveit.ros.org/)があります。

本セクションでは、MoveIt!を利用してマニピュレータによりブロックを移動します。

## マニピュレータとは

Describe what a manipulator is (robot made of links and joints used to move an
end effector to a desired position or along a desired path in order to achieve
some task).

## MoveIt!とは

MoveIt!はマニピュレータようのプラニングフレームワーク（「Motion Planning Framework」）です。主なユースケースは、グリッパーを指定された姿勢（位置と角度）への経路を計算し、マニピュレータのサーボの時間に対しての位置を制御し、グリッパーを移動させることです。

プラニングの一部として、周りのオブジェクトの位置やサイズを検討します。グリッパーだけでなくてマニピュレータ全体がオブジェクトに当たらないように経路を計算します。

MoveIt!は様々なロボットやタスクに利用されます。

<iframe width="560" height="315" src="https://www.youtube.com/embed/dblCGZzeUqs" frameborder="0" allowfullscreen></iframe>

## 必要なパッケージのインストール

すでに[Ubuntu LinuxとROSのインストール](linux_and_ros_install.html)のセクションで以下のコマンドによりMoveIt!をインストールしました。

```shell
sudo apt-get install ros-kinetic-moveit-*
```

MoveIt!はマニピュレータ一般的ようのソフトウェアです。利用する前にまずはマニピュレータをROSで利用可能化が必要です。そのためにCRANE+のROSパッケージをインストールします。

まずは新しいワークスペースを作成します。

```shell
$ cd ~/
$ mkdir -p ~/crane_plus_ws/src/
$ cd ~/crane_plus_ws/src/
$ catkin_init_workspace
Creating symlink "/home/geoff/crane_plus_ws/src/CMakeLists.txt" pointing to "/opt/ros/kinetic/share/catkin/cmake/toplevel.cmake"
$
```

次にCRANE+のROSパッケージをダウンロードしコンパイルします。

```shell
$ git clone git@github.com:gbiggs/crane_plus_arm.git
Cloning into 'crane_plus_arm'...
remote: Counting objects: 474, done.
remote: Total 474 (delta 0), reused 0 (delta 0), pack-reused 474
Receiving objects: 100% (474/474), 1.07 MiB | 1.09 MiB/s, done.
Resolving deltas: 100% (235/235), done.
Checking connectivity... done.
$ cd ~/crane_plus_ws/
$ catkin_make
Base path: /home/geoff/crane_plus_ws
Source space: /home/geoff/crane_plus_ws/src
Build space: /home/geoff/crane_plus_ws/build
Devel space: /home/geoff/crane_plus_ws/devel
Install space: /home/geoff/crane_plus_ws/install
（省略）
[ 80%] Built target crane_plus_arm_moveit_ikfast_plugin
[100%] Linking CXX executable /home/geoff/crane_plus_ws/devel/lib/crane_plus_camera_calibration/calibrate_camera_checkerboard
[100%] Built target calibrate_camera_checkerboard
$
```

ワークスペース内のパッケージが利用できるようにワークスペースをソースします。

```shell
$ source devel/setup.bash
```

これでCRANE+のパッケージが利用可能になり、ROSでCRANE+の利用が可能になりました。

## CRANE+のパッケージ

CRANE+のパッケージは一つだけではありません。複数のパッケージがそれぞれの持つ機能を合わせてマニピュレータを制御します。CRANE+用のパッケージを見てみましょう。

先クローンしたソースのディレクトリの中を見ると、以下のパッケージが見えます。

```shell
$ cd src/
$ ls
CMakeLists.txt  crane_plus_arm
$ cd crane_plus_arm/
$ ls
crane_plus_camera_calibration  crane_plus_gripper   crane_plus_ikfast_arm_plugin      crane_plus_moveit_config  crane_plus_simulation  README.md
crane_plus_description         crane_plus_hardware  crane_plus_joint_state_publisher  crane_plus_move_to_pose   LICENSE
```

`README.md`と`LICENSE`以外はすべてパッケージです。パッケージづつの機能を説明します。

`crane_plus_camera_calibration`
: CRANE+とカメラの位置関係を計算する

`crane_plus_description`
: ROSでなくてはならないCRANE+のURDF（Unified Robot Description Format、ROSのロボット定義フォーマット）モデルや、シミュレータ用のモデル

`crane_plus_gripper`
: CRANE+のグリッパーを制御するノード

`crane_plus_hardware`
: CRANE+のハードウェアと利用するために様々なノードの起動やパラメータ設定を行うlaunchファイル

`crane_plus_ikfast_arm_plugin`
: MoveIt!と利用するCRANE+のinverse kinematics（グリッパーの位置と角度を果たすためにジョイントのそれぞれの値の計算）プラグイン

`crane_plus_joint_state_publisher`
: Dynamixelサーボのコントローラが出力するサーボ状況メッセージ（`dynamixel_msgs/JointState`型） をROSの`sensor_msgs/JointState`型へ変換するノード

`crane_plus_moveit_config`
: CRANE+をMoveIt!で利用するためのパラメータやlaunchファイル等

`crane_plus_move_to_pose`
: MoveIt!を利用してCRANE+のグリッパーの姿勢をコマンドする簡単なツール

`crane_plus_simulation`
: `crane_plus_hardware`と類似なパッケージで、ハードウェアではなくてシミュレータ上でマニピュレータを操作するためのパラメータやlaunchファイル

基本的に`crane_plus_hardware`または`crane_plus_simulator`のlaunchファイルを利用してハードウェアかシミュレータのロボットを起動して、`crane_plus_moveit_config`のlaunchファイルでMoveIt!を起動したからロボットを制御します。次のセクションからこの手順を実習します。

## シミュレータのロボットを制御

ロボットハードウェアの制御には大きな弱点があります：制御ソフトウェアにエラーがあれば高いハードウェアを壊す可能性があります。

そのリスクを防ぐためにシミュレータの利用がおすすめです。ROSで[Gazebo](http://gazebosim.org/)というシミュレータの利用が基本です。

以下の実行でCRANE+のシミュレーションを起動します。

```shell
$ roslaunch crane_plus_simulation simulation.launch
```

![Simulated CRANE+](images/crane_plus_gazebo.png)

Gazeboでカメラの制御は以下で行います。

マウスをクリックとドラッグ
: クリックした点を中心にしてカメラをXZで移動する

__Shift__{: style="border: 1px solid black" } を押しながらマウスをクリックとドラッグ　または　マウスをミドルクリックとドラッグ
: クリックした点を中心にしてカメラの回転

マウスウィール　または　マウス右クリックとドラッグ
: クリックした点を中心にしてカメラズーム（注意：シミュレータ世界の点ので、遠いところにクリックしてズームするとカメラが急に遠くなる）

次にMoveIt!を起動します。新しい端末で以下を実行します。

```shell
$ cd ~/crane_plus_ws/
$ source devel/setup.bash
$ roslaunch crane_plus_moveit_config move_group.launch
... logging to /home/geoff/.ros/log/7b527712-3aa3-11e7-b868-d8cb8ae35bff/roslaunch-alnilam-3483.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://alnilam:33499/

SUMMARY
（省略）
[ INFO] [1494986092.617635076, 141.869000000]: MoveGroup context initialization complete

You can start planning now!
```

最後に「You can start planning now!」が出力されたら、マニピュレータは利用可能な状態になりました。

MoveIt!は、ROSノードでMoveIt!のAPIを利用することが基本の利用方法です。しかし、試すだけのためにノードを利用するや手動制御の時、ノードの利用は重い作業です。）ノードを作成の代わりにROSの基本の可視化ツール[RViz](http://wiki.ros.org/rviz)も利用できます。

MoveIt!はRViz上でマニピュレータ制御ユーザーインターフェースをプラグインとして提供します。MoveIt!と同時にインストールされて、CRANE+のMoveIt!パッケージから起動します。新しい端末で以下を実行してRVizのMoveIt!ユーザーインターフェースを起動します。

```shell
$ cd ~/crane_plus_ws/
$ source devel/setup.bash
$ roslaunch crane_plus_moveit_config moveit_rviz.launch
```

![MoveIt! RViz interface](images/crane_plus_moveit_rviz_panel_sim.png)

RVizでカメラの制御は以下で行います。

マウスをクリックとドラッグ
: 青い点を中心にしてカメラの回転

__Shift__{: style="border: 1px solid black" } を押しながらマウスをクリックとドラッグ　または　マウスをミドルクリックとドラッグ
: 青い点を中心にしてカメラをXYで移動する

マウスウィール　または　マウス右クリックとドラッグ
: 青い点を中心にしてカメラズーム

RViz内の「Motion Planning」パネルにある「Planning」タブをクリックして、以下のインターフェースを開きます。

![MoveIt! RViz interface](images/crane_plus_moveit_rviz_panel_planning_tab_sim_labeled.png)

最初のテストとして、マニピュレータをランダムな姿勢に移動しましょう。Planningタブ内の「Select Goal State」で「`<random valid>`」が選択されているを確認して、「Update」をクリックします。

![MoveIt! RViz random pose](images/crane_plus_moveit_rviz_random_pose_sim.png)

「Plan」をクリックします。MoveIt!が移動プランを計算します。RVizでロボットが追う経路は表示されます。

![MoveIt! RViz random pose plan](images/crane_plus_moveit_rviz_random_pose_plan_sim.png)

プランを実行します。「Execute」をクリックするとシミュレータ上のマニピュレータが指定した姿勢に移動します。RViz上でロボットの現在の姿勢が表示され、これも指定したポーズ（すなわちシミュレータ上のロボットのポーズ）に移動します。

![MoveIt! RViz random pose execution](images/crane_plus_moveit_rviz_random_pose_executed_sim.png)

![MoveIt! RViz random pose execution in the simulator](images/crane_plus_moveit_rviz_random_pose_executed_sim_gazebo.png)

ランダムな姿勢に移動できたら、次に手動制御を行いましょう。RViz内でマニピュレータの先端に球体と丸と矢印があります。以下の方法でこれらを利用してグリッパーの位置と角度が制御できます。

球体をドラッグ
: グリッパーの位置を移動する。

丸を回す
: グリッパーの角度を変更する。（注意：CRANE+は4DOFのマニピュレータだけなのでグリッパーの角度は上下（緑色の丸）しか変更できない。他の角度変更は無視される。）

矢印をドラッグ
: グリッパーの位置を一つの軸だけで移動する。

基本的にMoveIt!のユーザーインターフェースは可能か姿勢しか許さないので、時々マニピュレータは移動してくれないことや飛ぶことがあります。

お好みの姿勢にグリッパーを移動して、「Plan」と「Execute」ボタンでマニピュレータを移動しましょう。シミュレータ上のロボットはグリッパーが指定した位置と角度になるように動きます。

## ハードウェアのロボットを制御

CRANE+を制御するために、まずはROSとハードウェアのインターフェースになるノードを起動します。

CRANE+の電源を入れたから、以下を実行します。

```shell
$ roslaunch crane_plus_hardware start_arm_standalone.launch
```

エラー（赤い文字で表示される）がなければ、マニピュレータは起動しました。

次にMoveIt!を起動します。新しい端末で以下を実行します。

```shell
$ cd ~/crane_plus_ws/
$ source devel/setup.bash
$ roslaunch crane_plus_moveit_config move_group.launch
... logging to /home/geoff/.ros/log/7b527712-3aa3-11e7-b868-d8cb8ae35bff/roslaunch-alnilam-3483.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://alnilam:33499/

SUMMARY
（省略）
[ INFO] [1494986092.617635076, 141.869000000]: MoveGroup context initialization complete

You can start planning now!
```

最後に「You can start planning now!」が出力されたら、マニピュレータは利用可能な状態になりました。

MoveIt!は、ROSノードでMoveIt!のAPIを利用することが基本の利用方法です。しかし、試すだけのためにノードを利用するや手動制御の時、ノードの利用は重い作業です。）ノードを作成の代わりにROSの基本の可視化ツール[RViz](http://wiki.ros.org/rviz)も利用できます。

MoveIt!はRViz上でマニピュレータ制御ユーザーインターフェースをプラグインとして提供します。MoveIt!と同時にインストールされて、CRANE+のMoveIt!パッケージから起動します。新しい端末で以下を実行してRVizのMoveIt!ユーザーインターフェースを起動します。

```shell
$ cd ~/crane_plus_ws/
$ source devel/setup.bash
$ roslaunch crane_plus_moveit_config moveit_rviz.launch
```

![MoveIt! RViz interface](images/crane_plus_moveit_rviz_panel_hardware.png)

RVizでカメラの制御は以下で行います。

マウスをクリックとドラッグ
: 青い点を中心にしてカメラの回転

__Shift__{: style="border: 1px solid black" } を押しながらマウスをクリックとドラッグ　または　マウスをミドルクリックとドラッグ
: 青い点を中心にしてカメラをXYで移動する

マウスウィール　または　マウス右クリックとドラッグ
: 青い点を中心にしてカメラズーム

RViz内の「Motion Planning」パネルにある「Planning」タブをクリックして、以下のインターフェースを開きます。

![MoveIt! RViz interface](images/crane_plus_moveit_rviz_panel_planning_tab_hardware_labeled.png)

最初のテストとして、マニピュレータをランダムな姿勢に移動しましょう。Planningタブ内の「Select Goal State」で「`<random valid>`」が選択されているを確認して、「Update」をクリックします。

![MoveIt! RViz random pose](images/crane_plus_moveit_rviz_random_pose_hardware.png)

__注意：本物のロボットを制御します。ランダムで選択された姿勢は机等に当たらないようになるまでに、「Update」をクリックしましょう。__{: style="color: red" }

安全な姿勢になったら、「Plan」をクリックします。MoveIt!が移動プランを計算します。RVizでロボットが追う経路は表示されます。

![MoveIt! RViz random pose plan](images/crane_plus_moveit_rviz_random_pose_plan_hardware.png)

プランを実行します。「Execute」をクリックするとシミュレータ上のマニピュレータが指定した姿勢に移動します。RViz上でロボットの現在の姿勢が表示され、これも指定したポーズ（すなわちシミュレータ上のロボットのポーズ）に移動します。

![MoveIt! RViz random pose execution](images/crane_plus_moveit_rviz_random_pose_executed_hardware.png)

![CRANE+ random pose execution](images/crane_plus_random_pose_execution.jpg)

ランダムな姿勢に移動できたら、次に手動制御を行いましょう。RViz内でマニピュレータの先端に球体と丸と矢印があります。以下の方法でこれらを利用してグリッパーの位置と角度が制御できます。

球体をドラッグ
: グリッパーの位置を移動する。

丸を回す
: グリッパーの角度を変更する。（注意：CRANE+は4DOFのマニピュレータだけなのでグリッパーの角度は上下（緑色の丸）しか変更できない。他の角度変更は無視される。）

矢印をドラッグ
: グリッパーの位置を一つの軸だけで移動する。

基本的にMoveIt!のユーザーインターフェースは可能か姿勢しか許さないので、時々マニピュレータは移動してくれないことや飛ぶことがあります。

お好みの姿勢にグリッパーを移動して、「Plan」と「Execute」ボタンでマニピュレータを移動しましょう。シミュレータ上のロボットはグリッパーが指定した位置と角度になるように動きます。

## ノードからマニピュレータを制御

MoveIt!を利用するために、主にノードからアプリケーションやタスクに沿ったようにマニピュレータを制御したいでしょう。ここで簡単なノードの作成によりグリッパーの位置と角度を制御します。

最初にワークスペースにノード用の新しいパッケージを作成します。

```shell
$ cd ~/crane_plus_ws/src/
$ catkin_create_pkg gripper_mover roscpp moveit_core moveit_ros_planning_interface moveit_visual_tools moveit_msgs moveit_commander tf actionlib control_msgs geometry_msgs shape_msgs trajectory_msgs
Created file gripper_mover/CMakeLists.txt
Created file gripper_mover/package.xml
Created folder gripper_mover/include/gripper_mover
Created folder gripper_mover/src
Successfully created files in /home/geoff/crane_plus_ws/src/gripper_mover. Please adjust the values in package.xml. 
```

パッケージ内の`package.xml`の依存関係は以下のようになるように編集します。

```xml
  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>moveit_core</build_depend>
  <build_depend>moveit_ros_planning_interface</build_depend>
  <build_depend>moveit_visual_tools</build_depend>
  <build_depend>control_msgs</build_depend>
  <build_depend>actionlib</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>shape_msgs</build_depend>

  <run_depend>roscpp</run_depend>
  <run_depend>rospy</run_depend>
  <run_depend>moveit_core</run_depend>
  <run_depend>moveit_ros_planning_interface</run_depend>
  <run_depend>moveit_visual_tools</run_depend>
  <run_depend>tf</run_depend>
  <run_depend>geometry_msgs</run_depend>
  <run_depend>shape_msgs</run_depend>
  <run_depend>control_msgs</run_depend>
  <run_depend>actionlib</run_depend>
  <run_depend>trajectory_msgs</run_depend>
  <run_depend>moveit_msgs</run_depend>
  <run_depend>moveit_commander</run_depend>
```

パッケージ内の`CMakeLists.txt`にある`find_package`コマンドは以下の用に編集し依存関係パッケージを利用します。

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  control_msgs
  geometry_msgs
  shape_msgs
  actionlib
)

find_package(Boost REQUIRED
  system
  filesystem
  date_time
  thread
)
```

`CMakeLists.txt`の`catkin_package`コマンドにも追加します。

```cmake
catkin_package(
  CATKIN_DEPENDS
    roscpp
    control_msgs
    geometry_msgs
    shape_msgs
    moveit_core
    moveit_ros_planning_interface
    actionlib
)
```

`CMakeLists.txt`で`include_directories`にBoostのディレクトリを追加します。

```cmake
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)
```

パッケージ内の`CMakeLists.txt`にノード用のコンパイル情報を追加します。ここにもBoostの情報を利用します。

```cmake
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/servo_control_node.cpp)
add_executable(${PROJECT_NAME}_gripper_mover src/gripper_mover.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_gripper_mover PROPERTIES OUTPUT_NAME gripper_mover PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_gripper_mover ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_gripper_mover
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
```

なお、__必ず__{: style="color: red" } ファイルトップに`add_definitions(-std=c++11)`の行をアンコメントしてください。

`gripper_mover`パッケージ内の`src/`ディレクトリに`gripper_mover.cpp`というファイルを作成します。そしてエディターで開き、以下のソースを入力します。

```cpp
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "gripper_mover");
  ros::NodeHandle nh;

  ros::shutdown();
  return 0;
}
```

このソースは空のノードです。これから少しづつMoveIt!のAPIを利用するコードを追加してマニピュレータを制御します。

最初は、５行目（`ros::NodeHandle nh;`）の後に以下を追加します。MoveIt!はアシンクロナスな計算をしないといけないので、このコードによりROSのアシンクロナスな機能を初期化します。

```c++
  ros::AsyncSpinner spinner(1);
  spinner.start();
```

次はMoveIt!のAPIの初期化です。ファイルの上に以下のヘッダーをインクルードします。

```c++
#include <moveit/move_group_interface/move_group_interface.h>
```

そして`main`関数に以下の変数を追加します。

```c++
  moveit::planning_interface::MoveGroupInterface arm("arm");
```

MoveIt!は「MoveGroup」という存在を制御します。「MoveGroup」とは、ロボット内の複数のジョイントのグループです。CRANE+には２つのMoveGroupがあります。「arm」は手首の部分までのジョイントを制御し、「gripper」は指のジョイントのみを制御します。以下は「arm」のMoveGroupです。

![CRANE+ "arm" MoveGroup](images/crane_plus_move_group_arm.png)

`arm`のMoveGroupの先端はグリッパーの真ん中ぐらいに設定されています。これで`arm`の位置を指定すると、グリッパーはその位置の周りに行きます。

MoveIt!はどの座標系で制御するかを指定することが必要です。今回ロボットのベースに基づいた「`base_link`」座標系を利用します。位置制御の座標等はロボットのベースから図るという意味です。

```c++
  arm.setPoseReferenceFrame("base_link");
```

これでマニピュレータはもう制御できるようになりました。

最初の動きとして、マニピュレータを立てましょう。MoveIt!は「Named pose」（名付きポーズ）というコンセプトを持ちます。CRANE+のMoveIt!コンフィグレーション（`crane_plus_moveit_config`パッケージにある）は２つの名付きポーズを指定します。

`vertical`
: マニピュレータが真上を指す

`resting`
: マニピュレータは休んでいるような姿勢になる

`vertical`ポーズを利用してマニピュレータを立ちます。

```c++
  arm.setNamedTarget("vertical");
```

移動先を設定した後、MoveIt!にプラン作成を移動命令を出します。

```c++
  arm.move();
```
