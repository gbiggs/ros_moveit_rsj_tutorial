---
title: ROSの基本操作
date: 2017-06-16
---

- Table of contents
{:toc}

基本的なROS上で動くプログラムの書き方とビルド方法を学習します。

## 基本的な用語

パッケージ
: ノードや設定ファイル、コンパイル方法などをまとめたもの

ノード
: ROSの枠組みを使用する、実行ファイル

メッセージ
: ノード間でやりとりするデータ

トピック
: ノード間でメッセージをやりとりする際に、メッセージを置く場所

ノード、メッセージ、トピックの関係は以下の図のようにに表せます。

![ROS topics](images/ros-topic.png)

基本的には、ソフトウェアとしてのROSは、ノード間のデータのやりとりをサポートするための枠組みです。加えて、使い回しがきく汎用的なノードを、世界中のROS利用者で共有するコミュニティも、大きな意味でのROSの一部となっています。

## ソースコードを置く場所

ROSでは、プログラムをビルドする際に、catkin というシステムを使用しています。また、catkin は、 cmake というシステムを使っており、ROS用のプログラムのパッケージ毎に、cmakeの設定ファイルを作成することで、ビルドに必要な設定を行います。

以下の手順で本作業用の新しいワークスペースを作ります。

```shell
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
Creating symlink "/home/[ユーザダイレクトリー]/catkin/src/CMakeLists.txt"
    pointing to "/opt/ros/kinetic/share/catkin/cmake/toplevel.cmake"
$ ls
CMakeLists.txt
$ cd ..
$ ls
src
$
```

`catkin_ws`ディレクトリ内にある、`build`、`devel`は、catkinシステムがプログラムをビルドする際に使用するものなので、ユーザが触る必要はありません。`catkin_ws/src`ディレクトリは、ROSパッケージのソースコードを置く場所で、中にある`CMakeLists.txt` は、ワークスペース全体をビルドするためのルールが書かれているファイルです。

このディレクトリに、本作業ようのパッケージをダウンロードします。

```shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/gbiggs/rsj_tutorial_2017_ros_intro.git
$ ls
CMakeLists.txt  rsj_tutorial_2017_ros_intro
$
```

gitは、ソースコードなどの変更履歴を記録して管理する、分散型バージョン管理システムと呼ばれるものです。今回のセミナーでは詳細は触れませんが、研究開発を行う上では非常に有用なシステムですので、利用をお勧めします。公式の解説書、[Pro Git](https://git-scm.com/book/ja/v2")などを参考にして下さい。

GitHubは、ソースコードなどのリポジトリーサービスです。オープンソースソフトウェアの開発、共同作業及び配布を行うためによく利用されて、ROSではソースコードの保存と配布する場所としてもっとも人気なサービスです。バイナリーパッケージとして配布されているROSパッケージ以外の利用をする場合、GitHubを利用します。URLが分かれば上の手順だけで簡単にROSのパッケージが自分のワークスペースにインポートし利用することができます。

では、次にパッケージのディレクトリ構成を確認します。ダウンロードしているパッケージがバージョンアップされている場合などには、下記の実行例とファイル名が異なったり、ファイルが追加・削除されているが場合があります。

```shell
$ cd ~/catkin_ws/src/rsj_tutorial_2017_ros_intro/
$ ls
CMakeLists.txt  launch  msg  package.xml  src
$ ls launch/
say_hello.launch
$ ls msg/
Greeting.msg
$ ls src/
displayer.cpp  greeter.cpp
$
```

`CMakeLists.txt`と`package.xml`には、使っているライブラリの一覧や、生成する実行ファイルとC++のソースコードの対応など、このパッケージをビルドするために必要な情報が書かれています。`launch`ディレクトリには、複数のノードでできたシステムの定義が、`msg`ディレクトリには、このパッケージ独自のデータ形式の定義が、`src`ディレクトリには、このパッケージに含まれるプログラム(ノード)のソースコードが含まれています。

`catkin_make`コマンドで、ダウンとロードした`rsj_tutorial_2017_ros_intro`パッケージを含む、ワークスペース全体をビルドします。`catkin_make`は、ワークスペースの最上位ディレクトリ(`~/catkin_ws/`)で行います。

## ROSノードの理解とビルド・実行

先作成したワークスペースを利用します。端末を開き、パッケージが正しくあるか確認します。

```shell
$ cd ~/catkin_ws/src/
$ ls
CMakeLists.txt  rsj_tutorial_2017_ros_intro
$ cd ..
$
```

ソースファイルの編集にはお好みのテキストエディターが利用可能です。Linuxがはじめの方に`gedit`はおすすめです。

お好みのテキストエディターで`~/catkin_ws/src/rsj_tutorial_2017_ros_intro/src/greeter.cpp`を開きます。

![greeter.cpp](images/greeter_cpp_in_editor.png)

### 基本的なコードを読み解く

このコードが実行されたときの流れを確認しましょう。

まず、先頭部分では、必要なヘッダファイルをインクルードしています。

```c++
#include <ros/ros.h>
```

続いて、本ノードが利用するメッセージのヘッダファイルをインクルードしています。

```c++
#include <rsj_tutorial_2017_ros_basics/Greeting.h>
```

`std::string`が利用されるので、ヘッダファイルをインクルードします。

```c++
#include <string>
```

続いて、C++のmain関数が定義されています。本ノードは非常に簡単なのですべての機能がmain関数に入れられました。ただし、複雑な機能や色々なデータを持つノードにはクラスとしての実装がおすすめします。

```c++
int main(int argc, char **argv) {
  ros::init(argc, argv, "Greeter");
  ros::NodeHandle node;

  std::string hello_text;
  std::string world_name;
  ros::param::param<std::string>("~hello_text", hello_text, "hello");
  ros::param::param<std::string>("~world_name", world_name, "world");

  ros::Publisher pub = node.advertise<rsj_tutorial_2017_ros_basics::Greeting>("greeting", 1);

  ros::Rate rate(1);

  while (ros::ok()) {
    ros::spinOnce();

    ROS_INFO("Publishing greeting '%s %s'", hello_text, world_name);
    rsj_tutorial_2017_ros_basics::Greeting greeting;
    greeting.hello_text = hello_text;
    greeting.world_name = world_name;
    pub.publish(greeting);

    rate.sleep();
  }

  return 0;
}
```

`main`関数は、まずはノードのセットアップを行います。

`ros::init`はROSのインフラストラクチャの初期設定を行いノードを初期化します。1、2番目の引数には、main関数の引数をそのまま渡し、3番目の引数には、このノードの名前(この例では"Greeter")を与えます。

その次にある`ros::NodeHandle node`は、ノードを操るための変数を初期化します。

次の４行はパラメータの初期化です。ROSでは、純粋コマンドラインを利用するよりROSのパラメータ機能を利用することが標準的です。こうすると、コマンドラインだけではなくて、roslaunch（複数のノードを起動するためのツール）やGUIツールからもパラメータの設定が簡単にできます。

パラメータの初期化が終わったら、データ送信のためのパブリッシャーを初期化します。この変数の作成によりトピックが作成され、__このノードからデータの送信が可能になります。__{: style="color: red" } 以下の引数を与えています。

`"greeting"`
: トピック名：データをこのトピックに送信する

`1`
: メッセージのバッファリング量を指定 (大きくすると、処理が一時的に重くなったときなどに受け取り側の読み飛ばしを減らせる)

advertise関数についている、`<rsj_tutorial_2017_ros_basics::Greeting>`の部分は、メッセージの型を指定しています。これは、幾何的・運動学的な値を扱うメッセージを定義している`rsj_tutorial_2017_ros_basics`パッケージの、並進・回転速度を表す`Greeting`型です。(この指定方法は、C++のテンプレートという機能を利用していますが、ここでは、「`advertise`のときはメッセージの型指定を`<>`の中に書く、とだけ覚えておけば問題ありません。)

セットアップの最後として、`ros::Rate rate(1)`で周期実行のためのクラスを初期化しています。初期化時の引数で実行周波数(この例では1 Hz)を指定します。

`while(ros::ok())`で、メインの無限ループを回します（すなわちこのノードのメーンプロセッシングループです）。`ros::ok()`を`while`の条件にすることで、ノードの終了指示が与えられたとき(__Ctrl+c__{: style="border: 1px solid black" } が押された場合も含む)には、ループを抜けて終了処理などが行えるようになっています。

ループ中では、まず、_`ros::spinOnce()`を呼び出して、ROSのメッセージを受け取る_{: style="color: red" } といった処理を行います。`spinOnce`は、その時点で届いているメッセージの受け取り処理を済ませた後、すぐに処理を返します。`rate.sleep()`は、先ほど初期化した実行周波数を維持するように`sleep`します。

`ros::spinOnce()`と`rate.sleep()`の間に本ノードの処理を入れました。

最初は、ROSで情報を画面などに出力する際に用いる、ROS_INFO関数を呼び出してメッセージを表示しています。ほかにも、ROS_DEBUG、ROS_WARN、ROS_ERROR、ROS_FATAL関数が用意されています。

その後、データを送信します。まずは送信するデータ型（`rsj_tutorial_2017_ros_basics::Greeting`）を初期化し、値を設定します。先のセットアップで作成したパラメータの値を利用します。こうすると送信されるデータの内容は実行するときに自由に変更できます。

そして、`pub.publish(greeting)`によってデータを送信します。この行でデータはバッファーに入れられ、別のスレッドが自動的にサブスクライバに送信します(ROSのディフォルトはアシンクロナス送信です）。

メーンループが終了すると作成した変数は自動的にクリーンアップを実行しノードのシャットダウンを行います。

### ビルド＆実行

ROS上でこのパッケージをビルドするためには、`catkin_make`コマンドを用います。

```shell
$ cd ~/catkin_ws/
$ catkin_make
```

端末で実行してみましょう。

ROSシステムの実行の際、ROSを通してノード同士がデータをやりとりするために用いる、「roscore」を起動しておく必要があります。2つ目の端末を開き、それぞれで以下を実行して下さい。

1つ目の端末：

```shell
$ roscore
```

ROSでワークスペースを利用するとき、端末でそのワークスペースをアクティベートすることが必要です。このためにワークスペースの最上のディレクトリで`source devel/setup.bash`を実行します。このコマンドはワークスペースの情報を利用中の端末に読み込みます。しかし、 _仮のことだけ_ ので必ず新しい端末でワークスペースを利用し始めると _必ず_{: style="color: red"} まずは`source devel/setup.bash`を実行しなければなりません。一つの端末で一回だけ実行すれば十分です。その端末を閉じるまでに有効です。

2つ目の端末で下記を実行します。

```shell
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun rsj_tutorial_2017_ros_basics greeter
[ INFO] [1494840089.900580884]: Publishing greeting 'hello world'
```

上述が表示されれば成功です。

ソースコードにパラメータを利用したので、コマンドラインからパラメータの設定をためして見ましょう。ノードの端末（__注意：`roscore`の端末ではなくて__{: style="color: red" } ）に __Ctrl+c__{: style="border: 1px solid black" } を入力してノードを終了します。そして以下を実行してください。

```shell
$ rosrun rsj_tutorial_2017_ros_basics greeter _hello_text:=gidday _world_name:=planet
[ INFO] [1494840247.644756809]: Publishing greeting 'gidday planet'
```

両方の端末で __Ctrl+c__{: style="border: 1px solid black" } でノードと`roscore`を終了します。

### データを取得

以上のノードはデータを送信します。取得することももちろん可能です。

以下のソースは`rsj_tutorial_2017_ros_basics/src/displayer.cpp`ファイルにあります。

```c++
#include <ros/ros.h>
#include <rsj_tutorial_2017_ros_basics/Greeting.h>

#include <iostream>

void callback(const rsj_tutorial_2017_ros_basics::Greeting::ConstPtr &msg) {
  std::cout << msg->hello_text << " " << msg->world_name << '\n';
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "Displayer");
  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("greeting", 10, callback);

  ros::spin();

  return 0;
}
```

本ノードは`greeting`というトピックから取得したデータを端末に表示します。`greeter`からの差は以下のようです。

まずは`callback`関数です。この関数はトピックのデータ型に合っているポインターを引数としてもらいます。トピックからデータを受け取ったら、`callback`関数は呼ばれます。そのデータを`std::cout`に出力し終了します。

`const rsj_tutorial_2017_ros_basics::Greeting::ConstPtr` は、const型(内容を書き換えられない)、`rsj_tutorial_2017_ros_basics`パッケージに含まれる、`Greeting`型のメッセージの、const型ポインタを表しています。`&msg`の`&`は、参照型(内容を書き換えられるように変数を渡すことができる)という意味ですが、(const型なので)ここでは特に気にする必要はありません。msgはクラスへのポインタなので「-&gt;」を用い、以降はクラスのメンバ変数へのアクセスなので「.」を用いてアクセスしています。

`main`関数内にパラメータの初期化はなくなりました。本ノードはパラメータを利用しません。

`ros::Publisher`の初期化もなくなり、代わりに`ros::Subscriber`を初期化します。この行はトピックへのアクセスを初期化し、データが取得したらの対応を示します。引数は以下です。

`"greeting"`
: トピック名

`10`
: バッファーサイズ

`callback`
: メッセージを受け取ったときに呼び出す関数を指定 (`callback`関数)

最後に、メーンループはもういりません。本ノードはデータが届くとき以外何もしないので、無限ループになる`ros::spin()`を呼びます。`ros::spin()`は`greeter`の`while(...)`とros::spinOnce()`と類似の機能を中で持つので、ノードがシャットダウンされるまでに戻りません。

### ビルド＆実行

実行してみましょう。また1つ目の端末で以下を実行します。

```shell
$ roscore
```

そして2つ目の端末で以下を実行します。

```shell
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun rsj_tutorial_2017_ros_basics greeter
[ INFO] [1494840089.900580884]: Publishing greeting 'hello world'
```

最後に、3番目の端末を開いて、下記を実行します。

```shell
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun rsj_tutorial_2017_ros_basics displayer
```

「hello world」と表示されれば成功です。

以上の手順で、ROSパッケージに含まれるノードのソースコードを編集し、ビルドして、実行できるようになりました。

## システムとして扱おう

`roslaunch`を利用して、複数のノードでできたシステムをスタート・ストップします。

いつも手でノードを一つづつ起動することは面倒だけではなくて、エラーがちです。そのためにROSに`roslaunch`というツールがあります。`roslaunch`を利用するとシステム全体を一発で起動し、状況をモニターし、そして一発で泊めることが可能です。

### launchファイルを読み解く

launchファイルは、ノードやパラメータの組み合わせを定義するためのファイルです。 フォーマットはXMLです。システムに含まれるノードとそのノードの起動方法をを一つづつ定義します。

`rsj_tutorial_2017_ros_basics`パッケージに以下のファイルが`launch/say_hello.launch`として存在します。本ファイルは前セクションに手動で起動したシステムを定義します。

```xml
<launch>
  <node name="greeter" pkg="rsj_tutorial_2017_ros_basics" type="greeter">
    <param name="hello_text" value="allo"/>
    <param name="world_name" value="earth"/>
  </node>

  <node name="displayer" pkg="rsj_tutorial_2017_ros_basics" type="displayer" output="screen"/>
</launch>
```

`node`タグは２つあります。属性は下記の通りです。

`name`
: ノードインスタンスの名

`pkg`
: ノードを定義するパッケージ名

`type`
: ノードの実行ファイル名

`output`
: `stdout`の先：定義しないと`stdout`（`ROS_INFO`や`std::cout`への出力等）は端末で表示されず、`~/.ros/log/`に保存されるログファイルだけに出力される。

１番目の`<node>`は`greeter`ノードの定義です。本エレメントの中にパラメータの設定も行っています。パラメータの設定を行わない場合はノードのソースに定義したディフォルト値が利用されるので、必須ではありません。

２番目の`<node>`は`displayer`ノードの定義です。パラメータはありませんが、出力されることを端末で表示するようにします。

### roslaunchでシステムを起動

開いている端末に`roscore`や起動中のノードをすべて __Ctrl+c__{: style="border: 1px solid black" } で停止します。それから一つの端末で以下を実行します。

```shell
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch rsj_tutorial_2017_ros_basics say_hello.launch
... logging to /home/geoff/.ros/log/40887b56-395c-11e7-b868-d8cb8ae35bff/roslaunch-alnilam-11087.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://alnilam:40672/

SUMMARY
========

PARAMETERS
 * /greeter/hello_text: allo
 * /greeter/world_name: earth
 * /rosdistro: kinetic
 * /rosversion: 1.12.7

NODES
  /
    displayer (rsj_tutorial_2017_ros_basics/displayer)
    greeter (rsj_tutorial_2017_ros_basics/greeter)

auto-starting new master
process[master]: started with pid [11098]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 40887b56-395c-11e7-b868-d8cb8ae35bff
process[rosout-1]: started with pid [11111]
started core service [/rosout]
process[greeter-2]: started with pid [11118]
process[displayer-3]: started with pid [11126]
allo earth
allo earth
```

「allo earth」が繰り返して表示されたら成功です。

__Ctrl+c__{: style="border: 1px solid black" } でシステムを止めます。

```shell
allo earth
allo earth
allo earth
[Ctrl+c]
^C[displayer-3] killing on exit
[greeter-2] killing on exit
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
$
```

これでシステムのスタート・ストップが簡単になりました。

`roslaunch`を利用する場合は、別の端末で`roscore`の実行は不用です。`roslaunch`は中で`roscore`を起動したり停止したりします。

## ROSノードの作成

パッケージとノードを自分で作成しマニピュレータのサーボを操作します。

### パッケージを作成

ワークスペースに新しいパッケージを作成するために、以下を実行してください。

```shell
$ cd ~/catkin_ws/src
$ catkin_create_pkg servo_control roscpp dynamixel_controllers dynamixel_msgs
Created file servo_control/CMakeLists.txt
Created file servo_control/package.xml
Created folder servo_control/include/servo_control
Created folder servo_control/src
Successfully created files in /home/geoff/catkin_ws/src/servo_control. Please adjust the values in package.xml.
```

引数の１番目はパッケージ名です。２番目と３番目は依存パッケージの定義です。今回はC++で記述するため、`roscpp`に依存します。そしてDynamixelのサーボを利用するのでハードウェアとインターフェースする`dynamixel-controllers`に依存します。最後に、サーボコントローラにコマンドを送るためにDynamixel用のメッセージタイプの利用が必要ので、`dynamixel-msgs`に依存します。

生成されたパッケージの中身を確認します。

```shell
$ cd servo_control/
$ ls
CMakeLists.txt  include  package.xml  src
```

以前と同様に、`package.xml`はパッケージの情報を定義し、CMakeLists.txtはパッケージのビルド方法を定義します。

`package.xml`をエディターで開くと以下の行が含まれていると見えます。

```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>dynamixel_controllers</build_depend>
  <build_depend>dynamixel_msgs</build_depend>
  <build_depend>roscpp</build_depend>
  <run_depend>dynamixel_controllers</run_depend>
  <run_depend>dynamixel_msgs</run_depend>
  <run_depend>roscpp</run_depend>
```

これらはパッケージの依存を定義します。ROSとcatkinはこれらの利用によってパッケージのビルド順番等を決めます。

普段は`package.xml`のメール等（`<maintainer email>`タグ等）も編集するべきですが、今回は時間のために省略します。

以上、パッケージの作成ができました。

### ノードを作成

作成したパッケージにノードを作成します。

ノードを作成するために以下の手順を行います。

1. `CMakeLists.txt`にノードのコンパイル方法を追加する

1. ノードのソースファイルを作成する

#### CMakeLists.txtにノードを追加

`servo_control`パッケージにある`CMakeLists.txt`（`~/catkin_ws/src/servo_control/CMakeLists.txt`）をエディターで開き、以下の通りになるようにソースを編集します。（5行目、12行目、17行目および20行目を追加しました。ファイル内の133行目ぐらいから始まります。catkinのバージョンにより編集する具体的な行目の変更があります。）

```cmake
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/servo_control_node.cpp)
add_executable(${PROJECT_NAME}_set_servo_pos src/set_servo_pos.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
set_target_properties(${PROJECT_NAME}_set_servo_pos PROPERTIES OUTPUT_NAME set_servo_pos PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_set_servo_pos ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_set_servo_pos ${catkin_LIBRARIES})
```

なお、__必ず__{: style="color: red" } ファイルトップにある`add_definitions(-std=c++11)`の行をアンコメントしてください。

これでcatkinに`set_servo_pos`というノードのコンパイルを指定しました。

#### ノードのソースの作成

`servo_control`パッケージ内の`src/`ディレクトリに`set_servo_pos.cpp`というファイル（`~/catkin_ws/src/servo_control/src/set_servo_pos.cpp`）を作成します。そしてエディターで開き、以下のソースを入力します。

```c++
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <string>
#include <vector>

int main(int argc, char **argv) {
  ros::init(argc, argv, "set_servo_pos");
  ros::NodeHandle node;

  std::string servo_command_topic;
  ros::param::param<std::string>(
    "~servo_command_topic",
    servo_command_topic,
    "/finger_servo_controller/command");

  ros::Publisher pub = node.advertise<std_msgs::Float64>(servo_command_topic, 1);

  std::vector<std::string> my_args;
  ros::removeROSArgs(argc, argv, my_args);
  if (my_args.size() < 2) {
    ROS_FATAL("Usage: rosrun set_servo_pos [position]");
    ros::shutdown();
    return 1;
  }
  std_msgs::Float64 servo_pos;
  servo_pos.data = std::stof(my_args[1]);

  ROS_INFO("Setting servo position to %f", servo_pos.data);
  ros::Rate r(1);
  while (ros::ok()) {
    pub.publish(servo_pos);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
```

このソースはコマンドラインから希望のサーボの位置を読み、パラメータで指定されたトピック（ディフォルトは`/finger_servo_controller/command`）にパブリッシュします。このトピックはサーボのコントローラがサブスクライブしています。

なぜサーボの位置はパラメータとしてノードに渡さなかったでしょうか。パラメータはいわゆる「configuration」です。ノードの振る舞いを制御するためのことです。サーボのトピックははノードの振る舞えだが、サーボの位置はコマンドです。このノードはコマンドラインで利用するべきなツールなので普通のコマンドラインパラメータを利用しました。

### ビルド＆実行

パッケージ内のソースを変更・追加した後、必ず再ビルドが必要です。端末でcatkin_makeコマンドを実行して再ビルドします。

1つ目の端末：

```shell
$ cd ~/catkin_ws/
$ catkin_make
```

この際、_ビルドエラーが出ていないか、良く確認して下さい_{: style="color: red" }。エラーが出ている場合は、ソースコードの該当箇所を確認・修正して下さい。

実行する前にまずはマニピュレータが壊れないようにします。マニピュレータのグリッパーが動くので、__電源を入れる前にマニピュレータのグリッパーは何にもぶつからないような姿勢にしましょう__{: style="color: red" } 。

マニピュレータのサーボコントローラを起動することが必要です。

`servo_controller`パッケージの中に以下のファイルを作成します。`~/catkin_ws/src/servo_control/config`と`~/catkin_ws/src/servo_control/launch`ディレクトリは作成した上でファイルを編集します。

```shell
$ cd ~/catkin_ws/src/servo_control
$ mkdir config
$ mkdir launch
```

`~/catkin_ws/src/servo_control/config/dynamixel_test.yaml`:

```yaml
finger_servo_controller:
    controller:
        package: dynamixel_controllers
        module: joint_position_controller
        type: JointPositionController
    joint_name: finger_joint
    joint_speed: 1.17
    motor:
        id: 5
        init: 512
        min: 0
        max: 1023
```

`~/catkin_ws/src/servo_control/launch/dynamixel_test.launch`:

```xml
<launch>
    <node name="dynamixel_manager" pkg="dynamixel_controllers"
      type="controller_manager.py" required="true" output="screen">
        <rosparam>
            namespace: dynamixel_controller_manager
            serial_ports:
                dxl_tty1:
                    port_name: "/dev/ttyUSB0"
                    baud_rate: 1000000
                    min_motor_id: 1
                    max_motor_id: 5
                    update_rate: 10
        </rosparam>
    </node>
    <rosparam file="$(find servo_control)/config/dynamixel_test.yaml" command="load"/>
    <node name="finger_servo_spawner" pkg="dynamixel_controllers" type="controller_spawner.py"
          args="--manager=dynamixel_controller_manager
                --port dxl_tty1
                finger_servo_controller"
          output="screen"/>
</launch>
```

1つ目の端末に以下を実行してマニピュレータのグリッパーサーボコントローラを起動します。

```shell
$ roslaunch servo_control dynamixel_test.launch
... logging to /home/geoff/.ros/log/619c447c-396a-11e7-b868-d8cb8ae35bff/roslaunch-alnilam-1790.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://alnilam:44912/

SUMMARY
========
（省略）
```

2つ目の端末に以下を実行します。

```shell
$ rosrun servo_control set_servo_pos 0
[ INFO] [1494851539.189274395]: Setting servo position to 0.000000
[Ctrl+cで止める]
$ rosrun servo_control set_servo_pos -0.5
[ INFO] [1494851548.085785357]: Setting servo position to -0.500000
[Ctrl+cで止める]
```

サーボが動けば、サーボ制御は成功しました。

_このソースは以下のURLでダウンロード可能です。_

<https://github.com/gbiggs/rsj_2017_servo_control>

## サーボの状態を確認

もう一つのノードを作成し、サーボの現在状況を端末で表示します。上記と同じ手順で`servo_status`というノードを`servo_control`パッケージに追加します。

### ノードを作成

#### CMakeLists.txtにノードを追加

`~/catkin_ws/src/servo_control/CMakeLists.txt`を編集して、新しいノードのコンパイル方法をしていします。ファイルをエディターで開き以下の通りになるように編集します。
（6行目、14行目、20行目および24行目を追加しました。）

```cmake
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/servo_control_node.cpp)
add_executable(${PROJECT_NAME}_set_servo_pos src/set_servo_pos.cpp)
add_executable(${PROJECT_NAME}_servo_status src/servo_status.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
set_target_properties(${PROJECT_NAME}_set_servo_pos PROPERTIES OUTPUT_NAME set_servo_pos PREFIX "")
set_target_properties(${PROJECT_NAME}_servo_status PROPERTIES OUTPUT_NAME servo_status PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_set_servo_pos ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_servo_status ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_set_servo_pos ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_servo_status ${catkin_LIBRARIES})
```

#### ノードのソースの作成

`servo_control`パッケージ内の`src/`ディレクトリに`servo_status.cpp`というファイルを作成します。エディターで`~/catkin_ws/src/servo_control/src/servo_status.cpp`を開き（作成）、以下のソースを入力します。

```c++
#include <ros/ros.h>
#include <dynamixel_msgs/JointState.h>

#include <string>

void callback(const dynamixel_msgs::JointState::ConstPtr &msg) {
  ROS_INFO("--- Servo status ---");
  ROS_INFO("Name: %s", msg->name.c_str());
  ROS_INFO("ID: %d", msg->motor_ids[0]);
  ROS_INFO("Temperature: %d", msg->motor_temps[0]);
  ROS_INFO("Goal position: %f", msg->goal_pos);
  ROS_INFO("Current position: %f", msg->current_pos);
  ROS_INFO("Position error: %f", msg->error);
  ROS_INFO("Velocity: %f", msg->velocity);
  ROS_INFO("Load: %f", msg->load);
  ROS_INFO("Moving: %s", msg->is_moving ? "yes" : "no");
  ROS_INFO("");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "servo_status");
  ros::NodeHandle node;

  std::string servo_status_topic;
  ros::param::param<std::string>(
    "~servo_status_topic",
    servo_status_topic,
    "/finger_servo_controller/state");

  ros::Subscriber pub = node.subscribe<dynamixel_msgs::JointState>(
    servo_status_topic,
    10,
    callback);

  ros::spin();

  return 0;
}
```

コールバックの中に`dynamixel_msgs::JointState`というメッセージタイプを利用します。このメッセージタイプは`dynamixel_msgs`パッケージに定義され、内容は以下のようです。

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string name
int32[] motor_ids
int32[] motor_temps
float64 goal_pos
float64 current_pos
float64 error
float64 velocity
float64 load
bool is_moving
```

### ビルド＆実行

コンパイルして実行します。1つ目の端末で`catkin_make`を実行します。

```shell
$ cd ~/catkin_ws/
$ catkin_make
```

実行する前にマニピュレータのサーボコントローラを起動することが必要です。1つ目の端末に下記を実行してマニピュレータのグリッパーサーボコントローラを起動します。

```shell
$ source devel/setup.bash
$ roslaunch servo_control dynamixel_test.launch
... logging to /home/geoff/.ros/log/619c447c-396a-11e7-b868-d8cb8ae35bff/roslaunch-alnilam-1790.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://alnilam:44912/

SUMMARY
========
（省略）
```

2つ目の端末に下記を実行します。

```shell
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun servo_control servo_status
[ INFO] [1494855697.336794278]: --- Servo status ---
[ INFO] [1494855697.336922059]: Name: finger_joint
[ INFO] [1494855697.336968787]: ID: 5
[ INFO] [1494855697.337016662]: Temperature: 37
[ INFO] [1494855697.337069086]: Goal position: 0.000000
[ INFO] [1494855697.337120585]: Current position: 0.000000
[ INFO] [1494855697.337170811]: Position error: 0.000000
[ INFO] [1494855697.337226948]: Velocity: 0.000000
[ INFO] [1494855697.337278499]: Load: 0.000000
[ INFO] [1494855697.337329531]: Moving: no
[ INFO] [1494855697.337372008]:
[ INFO] [1494855697.432684658]: --- Servo status ---
（省略）
```

（サーボの現在状況により数字の変わることがあります。）

3つ目の端末に下記を実行すると、`servo_status`の端末で数字の変更が見えます。

```shell
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun servo_control set_servo_pos 0
[ INFO] [1494851539.189274395]: Setting servo position to 0.000000
[Ctrl+cで止める]
$ rosrun servo_control set_servo_pos -0.5
[ INFO] [1494851548.085785357]: Setting servo position to -0.500000
[Ctrl+cで止める]
```

_このソースは以下のURLでダウンロード可能です。_

<https://github.com/gbiggs/rsj_2017_servo_control>

## 小課題

サーボステータスに`error`と`load`と`is_moving`という値があります。これらの利用により、サーボはストールしたかどうか判断できます。

サーボがストールしたら警告を端末で表示するノードを作成してみましょう。
