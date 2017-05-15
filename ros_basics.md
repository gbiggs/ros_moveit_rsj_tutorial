---
layout: article
title: ROSの基本操作
date: 2017-05-09
---

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

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
Creating symlink "/home/[ユーザーダイレクトリー]/catkin/src/CMakeLists.txt" pointing to "/opt/ros/kinetic/share/catkin/cmake/toplevel.cmake"
$ ls
CMakeLists.txt
$ cd ..
$ ls
src
$
```

`catkin_ws`ディレクトリの中にある、`build`、`devel`は、catkinシステムがプログラムをビルドする際に使用するものなので、ユーザが触る必要はありません。`catkin_ws/src`ディレクトリは、ROSパッケージのソースコードを置く場所で、中にある`CMakeLists.txt` は、ワークスペース全体をビルドするためのルールが書かれているファイルです。

このディレクトリに、本作業ようのパッケージをダウンロードします。

```
$ git clone https://github.com/gbiggs/rsj_tutorial_2017_ros_intro.git
$ ls
CMakeLists.txt  rsj_tutorial_2017_ros_intro
$
```

gitは、ソースコードなどの変更履歴を記録して管理する、分散型バージョン管理システムと呼ばれるものです。今回のセミナーでは詳細は触れませんが、研究開発を行う上では非常に有用なシステムですので、利用をお勧めします。公式の解説書、[Pro Git](https://git-scm.com/book/ja/v2")などを参考にして下さい。

GitHubは、ソースコードなどのリポジトリーサービスです。オープンソースソフトウェアの開発、共同作業及び配布を行うためによく利用されて、ROSではソースコードの保存と配布する場所としてもっとも人気なサービスです。バイナリーパッケージとして配布されているROSパッケージ以外の利用をする場合、GitHubを利用します。URLが分かれば上の手順だけで簡単にROSのパッケージが自分のワークスペースにインポートし利用することができます。

では、次にパッケージのディレクトリ構成を確認します。ダウンロードしているパッケージがバージョンアップされている場合などには、下記の実行例とファイル名が異なったり、ファイルが追加・削除されているが場合があります。

```
$ cd rsj_tutorial_2017_ros_intro/
$ ls
CMakeLists.txt  launch  msg  package.xml  src
$ ls launch/
say_hello.launch
$ ls msg/
Greeting.msg
$ ls src/
greeter.cpp
$
```

`CMakeLists.txt`と`package.xml`には、使っているライブラリの一覧や、生成する実行ファイルとC++のソースコードの対応など、このパッケージをビルドするために必要な情報が書かれています。`launch`ディレクトリには、複数のノードでできたシステムの定義が、`msg`ディレクトリには、このパッケージ独自のデータ形式の定義が、`src`ディレクトリには、このパッケージに含まれるプログラム(ノード)のソースコードが含まれています。

以下のように、`catkin_make`コマンドで、ダウンとロードした`rsj_tutorial_2017_ros_intro`パッケージを含む、ワークスペース全体をビルドします。`catkin_make`は、ワークスペースの最上位ディレクトリ(`~/catkin_ws/`)で行います。

```
$ cd ~/catkin_ws/
```

## ROSノードの理解とビルド・実行

先作成したワークスペースを利用します。端末を開き、パッケージが正しくあるか確認します。

```
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

```
#include <ros/ros.h>
```

続いて、本ノードが利用するメッセージのヘッダファイルをインクルードしています。

```
#include <rsj_tutorial_2017_ros_basics/Greeting.h>
```

`std::string`が利用されるので、ヘッダファイルをインクルードします。

```
#include <string>
```

続いて、C++のmain関数が定義されています。本ノードは非常に簡単なのですべての機能がmain関数に入れられました。ただし、複雑な機能や色々なデータを持つノードにはクラスとしての実装がおすすめします。

```
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

パラメータの初期化が終わったら、データ送信のためのパブリッシャーを初期化します。この変数の作成によりトピックが作成され、このノードからデータの送信が可能になります。

セットアップの最後として、`ros::Rate rate(1)`で周期実行のためのクラスを初期化しています。初期化時の引数で実行周波数(この例では1 Hz)を指定します。

`while(ros::ok())`で、メインの無限ループを回します（すなわちこのノードのメーンプロセッシングループです）。`ros::ok()`を`while`の条件にすることで、ノードの終了指示が与えられたとき(__Ctrl+c__{: style="border: 1px solid black" } が押された場合も含む)には、ループを抜けて終了処理などが行えるようになっています。

ループ中では、まず、_`ros::spinOnce()`を呼び出して、ROSのメッセージを受け取る_{: style="color: red" } といった処理を行います。`spinOnce`は、その時点で届いているメッセージの受け取り処理を済ませた後、すぐに処理を返します。`rate.sleep()`は、先ほど初期化した実行周波数を維持するように`sleep`します。

`ros::spinOnce()`と`rate.sleep()`の間に本ノードの処理を入れました。

最初は、ROSで情報を画面などに出力する際に用いる、ROS_INFO関数を呼び出してメッセージを表示しています。ほかにも、ROS_DEBUG、ROS_WARN、ROS_ERROR、ROS_FATAL関数が用意されています。

その後、データを送信します。まずは送信するデータ型（`rsj_tutorial_2017_ros_basics::Greeting`）を初期化し、値を設定します。先のセットアップで作成したパラメータの値を利用します。こうすると送信されるデータの内容は実行するときに自由に変更できます。

そして、`pub.publish(greeting)`によってデータを送信します。この行でデータはバッファーに入れられ、別のスレッドが自動的にサブスクライバに送信します(ROSのディフォルトはアシンクロナス送信です）。

メーンループが終了すると作成した変数は自動的にクリーンアップを実行しノードのシャットダウンを行います。

### ビルド＆実行

ROS上でこのパッケージをビルドするためには、catkin_makeコマンドを用います。

```
$ cd ~/catkin_ws/
$ catkin_make
```

実行してみましょう。実行の際、ROSを通してノード同士がデータをやりとりするために用いる、「roscore」を起動しておく必要があります。2つの端末を開き、それぞれで以下を実行して下さい。

```
$ roscore
```

```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun rsj_tutorial_2017_ros_basics greeter
[ INFO] [1494840089.900580884]: Publishing greeting 'hello world'
```

上述が表示されれば成功です。

ソースコードにパラメータを利用したので、コマンドラインからパラメータの設定をためして見ましょう。ノードの端末（__注意：`roscore`の端末ではなくて__{: style="color: red" } ）に__Ctrl+c__{: style="border: 1px solid black" } を入力してノードを終了します。そして以下を実行してください。

```
$ rosrun rsj_tutorial_2017_ros_basics greeter _hello_tex=gidday _world_name:=planet
[ INFO] [1494840247.644756809]: Publishing greeting 'gidday planet'
```

### データを取得

以上のノードはデータを送信します。取得することももちろん可能です。

以下のソースは`rsj_tutorial_2017_ros_basics/src/displayer.cpp`ファイルにあります。

```
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

まずは`callback`関数です。この関数はトピックのデータ型に合っているポインターを引数としてもらいます。トピックにデータが届いたら、`callback`関数は呼ばれます。そのデータを`std::cout`に出力し終了します。

`main`関数内にパラメータの初期化はなくなりました。本ノードはパラメータを利用しません。

`ros::Publisher`の初期化もなくなり、代わりに`ros::Subscriber`を初期化します。この行はトピックへのアクセスを初期化し、データが取得したらの対応を示します。１番目の引数はトピック名で、２番目はバッファーサイズで、３番目はデータが届くと呼ぶ関数です。

最後に、メーンループはもういりません。本ノードはデータが届くとき以外何もしないので、無限ループになる`ros::spin()`を呼びます。`ros::spin()`は`greeter`の`while(...)`とros::spinOnce()`と類似の機能を中で持つので、ノードがシャットダウンされるまでに戻りません。

### ビルド＆実行

もう一回、このパッケージをビルドするためにcatkin_makeコマンドを実行します。

```
$ cd ~/catkin_ws/
$ catkin_make
```

実行してみましょう。また端末で以下を実行します。

```
$ roscore
```

```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun rsj_tutorial_2017_ros_basics greeter
[ INFO] [1494840089.900580884]: Publishing greeting 'hello world'
```

そしてもう一つの端末を開いて、以下を実行します。

```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun rsj_tutorial_2017_ros_basics displayer

「hello world」と表示されれば成功です。

以上の手順で、ROSパッケージに含まれるノードのソースコードを編集し、ビルドして、実行できるようになりました。

## システムとして扱おう

`roslaunch`を利用して、複数のノードでできたシステムをスタート・ストップします。

いつも手でノードを一つづつ起動することは面倒だけではなくて、エラーがちです。そのためにROSに`roslaunch`というツールがあります。`roslaunch`を利用するとシステム全体を一発で起動し、状況をモニターし、そして一発で泊めることが可能です。

### launchファイルを読み解く

launchファイルは、ノードやパラメータの組み合わせを定義するためのファイルです。 フォーマットはXMLです。システムに含まれるノードとそのノードの起動方法をを一つづつ定義します。

`rsj_tutorial_2017_ros_basics`パッケージに以下のファイルが`launch/say_hello.launch`として存在します。本ファイルは前セクションに手動で起動したシステムを定義します。

```
<launch>
  <node name="greeter" pkg="rsj_tutorial_2017_ros_basics" type="greeter">
    <param name="hello_text" value="allo"/>
    <param name="world_name" value="earth"/>
  </node>

  <node name="displayer" pkg="rsj_tutorial_2017_ros_basics" type="displayer" output="screen"/>
</launch>
```

`node`エレメントは２つあります。アトリビュートは以下です。

`name`
: ノードインスタンスの名

`pkg`
: ノードを定義するパッケージ

`type`
: ノードの実行ファイル名

`output`
: `stdout`の先：定義しないと`stdout`（`ROS_INFO`や`std::cout`への出力等）は端末で表示されず、`~/.ros/log/`に保存されるログファイルだけに出力される。

１番目の`<node>`は`greeter`ノードの定義です。本エレメントの中にパラメータの設定も行っています。パラメータの設定を行わない場合はノードのソースに定義したディフォルト値が利用されるので、必須ではありません。

２番目の`<node>`は`displayer`ノードの定義です。パラメータはありませんが、出力されることを端末で表示するようにします。

### roslaunchでシステムを起動

開いている端末に`roscore`や起動中のノードをすべて __Ctrl+c__{: style="border: 1px solid black" } で止めます。それから一つの端末で以下を実行します。

```
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

```
[Ctrl+c]
allo earth
allo earth
allo earth
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

## ROSノードの作成

パッケージとソースコードを自分で作成してノードを作成し、マニピュレータのサーボを操作します。

### パッケージを作成

まずは、ワークスペースにマニピュレータのハードウェアとインターフェースするソフトウェアを含めます。

ワークスペースに新しいパッケージを作成するために、以下を実行してください。

$ cd ~/catkin_ws/src
$ catkin_create_pkg servo_control roscpp 

### ノードを作成

### ビルド＆実行

## ロボットに速度指令を与える

先ほどのひな形を編集して、ロボットを動かします。以下の作業は、QtCreator上でソースコードを編集します。

### ロボットに速度指令を与えるコードを追加

まず、ロボットに速度指令(目標並進速度・角速度)を与えるコードを追加します。ひな形には既に、速度指令値が入ったメッセージを出力するための初期化コードが含まれていますので、この部分の意味を確認します。

```
rsj_robot_test_node():
{
  ros::NodeHandle nh("~");
  pub_twist = nh.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 5);
  sub_odom = nh.subscribe("/ypspur_ros/odom", 5, &rsj_robot_test_node::cb_odom, this);
}
```

ソースコード中の、rsj_robot_test_nodeクラスの、rsj_robot_test_node関数は、クラスのコンストラクタと呼ばれるもので、_クラスが初期化されるときに自動的に呼び出されます_{: style="color: red" }。この中で、`nh.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 5);`の部分で、_このノードが、これからメッセージを出力する_{: style="color: red" } ことを宣言しています。advertise関数に与えている引数は以下のような意味を持ちます。

`"/ypspur_ros/cmd_vel"`
: 出力するメッセージを置く場所(トピックと呼ぶ)を指定

`5`
: メッセージのバッファリング量を指定 (大きくすると、処理が一時的に重くなったときなどに受け取り側の読み飛ばしを減らせる)

advertise関数についている、&lt;geometry_msgs::Twist&gt;の部分は、メッセージの型を指定しています。これは、幾何的・運動学的な値を扱うメッセージを定義しているgeometry_msgsパッケージの、並進・回転速度を表すTwist型です。(この指定方法は、C++のテンプレートという機能を利用していますが、ここでは、「advertiseのときはメッセージの型指定を&lt;&gt;の中に書く、とだけ覚えておけば問題ありません。)

以下のコードを、mainloop関数の中(「ここに速度指令の出力コード」の部分)に入れることで、速度指令のメッセージを出力(publish)します。

```
geometry_msgs::Twist cmd_vel;
cmd_vel.linear.x = 0.05;
cmd_vel.angular.z = 0.0;
pub_twist.publish(cmd_vel);
```

### ビルド＆実行

  $ cd ~/catkin_ws/
  $ catkin_make

この際、_ビルドエラーが出ていないか、良く確認して下さい_{: style="color: red" }。エラーが出ている場合は、ソースコードの該当箇所を確認・修正して下さい。

実行の際、まずroscoreと、ypspur_rosを起動します。ypspur_rosの中では、ロボットの動作テストの際に使用した、ypspur-coordinatorが動いています。なお、roscoreは、前のものを実行し続けている場合は、そのままで使用できます。コマンド入力の際は、タブ補完を活用しましょう。

```
$ roscore
```

```
$ rosrun ypspur_ros ypspur_ros _param_file:=/home/ubuntu/params/rsj-seminar20??.param [該当するものに置き換えること] _port:=/dev/serial/by-id/usb-T-frog_project_T-frog_Driver-if00
```

続いて、別の端末でrsj_robot_test_nodeノードを実行します。まずは、ロボットのホイールを浮かせて、走り出さない状態にして実行してみましょう。

```
$ rosrun rsj_robot_test rsj_robot_test_node
Hello ROS World!
```

ゆっくりとホイールが回れば、正しく動作しています。__Ctrl+c__{: style="border: 1px solid black" } で終了します。

### 小課題

速度、角速度を変更して動作を確認してみましょう。

## ロボットの状態を表示する

### ロボットの状態を表示するコードを追加

まず、ロボットの動作したときの移動量やオドメトリ座標を取得、表示するコードを追加します。ひな形には既に、移動量や座標が入ったメッセージを受け取るコードが含まれていますので、この部分の意味を確認します。

```
rsj_robot_test_node():
{
  ros::NodeHandle nh("~");
  pub_twist = nh.advertise<geometry_msgs::Twist>( "/ypspur_ros/cmd_vel", 5);
  sub_odom = nh.subscribe("/ypspur_ros/odom", 5, &rsj_robot_test_node::cb_odom, this);
}
```

この中で、

```
nh.subscribe("/ypspur_ros/odom", 5, &rsj_robot_test_node::cb_odom, this);
```

の部分で、_このノードが、これからメッセージを受け取る_{: style="color: red" } ことを宣言しています。subscribe関数に与えている引数は以下のような意味を持ちます。

`"/ypspur_ros/odom"`
: 受け取るメッセージが置かれている場所(トピック)を指定

`5`
: メッセージのバッファリング量を指定 (大きくすると、処理が一時的に重くなったときなどに読み飛ばしを減らせる)

`&rsj_robot_test_node::cb_odom`
: メッセージを受け取ったときに呼び出す関数を指定 (rsj_robot_test_nodeクラスの中にある、cb_odom関数)

`this`
: メッセージを受け取ったときに呼び出す関数がクラスの中にある場合にクラスの実体を指定 (とりあえず、おまじないと思って構いません。)

これにより、rsj_robot_test_nodeノードは、/odomトピックからメッセージをうけとると、cb_odom関数が呼び出されるようになります。続いてcb_odom関数の中身を確認しましょう。

```
void cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
}
```

`const nav_msgs::Odometry::ConstPtr` は、const型(内容を書き換えられない)、nav_msgsパッケージに含まれる、Odometry型のメッセージの、const型ポインタを表しています。`&msg`の`&`は、参照型(内容を書き換えられるように変数を渡すことができる)という意味ですが、(const型なので)ここでは特に気にする必要はありません。

cb_odom関数に、以下のコードを追加してみましょう。これにより、受け取ったメッセージの中から、ロボットの並進速度を取り出して表示できます。

```
ROS_INFO("vel %f", msg->twist.twist.linear.x);
```

ここで、`msg->twist.twist.linear.x` の意味を確認します。`nav_msgs::Odometry`メッセージには、下記のように入れ子状にメッセージが入っています。

```
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

全て展開すると、以下の構成になります。

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x // ロボット並進速度
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z // ロボット角速度
  float64[36] covariance
```

読みたいデータである、ロボット並進速度を取り出すためには、これを順にたどっていけば良く、`msg->twist.twist.linear.x`{: style="color: red" } 、となります。msgはクラスへのポインタなので「-&gt;」を用い、以降はクラスのメンバ変数へのアクセスなので「.」を用いてアクセスしています。

### ビルド＆実行

```
$ cd ~/catkin_ws/
$ catkin_make
```

この際、ビルドエラーが出ていないか、良く確認して下さい。エラーが出ている場合は、ソースコードの該当箇所を確認・修正して下さい。

まず、先ほどと同様、roscoreと、ypspur_rosを起動します。(以降、この手順の記載は省略します。)

```
$ roscore
```

```
$ rosrun ypspur_ros ypspur_ros _param_file:=/home/ubuntu/params/rsj-seminar20??.param [該当するものに置き換えること] _port:=/dev/serial/by-id/usb-T-frog_project_T-frog_Driver-if00
```

続いて、rsj_robot_test_nodeノードを実行します。

```
$ rosrun rsj_robot_test rsj_robot_test_node
Hello ROS World!
vel: 0.0500
vel: 0.0500
vel: 0.0500
vel: 0.0500
```

ロボットのホイールが回転し、先ほどの小課題で設定した走行指令の値と近い値が表示されれば、正しく動作しています。

### 小課題

同様に、ロボットの角速度を表示してみましょう。


## シーケンス制御

### 時間で動作を変える

メインループを以下のように変更してみましょう。

```
void mainloop()
{
  ROS_INFO("Hello ROS World!");

  ros::Rate rate(10.0);
  ros::Time start = ros::Time::now();
  while(ros::ok())
  {
    ros::spinOnce();
    ros::Time now = ros::Time::now();

    geometry_msgs::Twist cmd_vel;
    if(now - start &gt; ros::Duration(3.0))
    {
      cmd_vel.linear.x = 0.05;
      cmd_vel.angular.z = 0.0;
    }
    pub_twist.publish(cmd_vel);

    rate.sleep();
  }
}
```

これは、メインループ開始時刻から、3.0秒後に、並進速度0.05m/sの指令を与えるコードです。ros::Time型(時刻を表す)同士の減算結果は、ros::Duration型(時間を表す)になり、比較演算子で比較できます。したがって、now - start > ros::Duration(3.0)の部分は、開始から3秒後に、trueになります。

先ほどと同様にビルドし、ypspur_rosとrsj_robot_test_nodeを起動して動作を確認します。

### センシング結果で動作を変える

cb_odomで取得したオドメトリのデータを保存しておくように、以下のように変更してみましょう。(`// 追加`{: style="color: red" } の部分を追加)

```
void cb_odom(const nav_msgs::Odometry::ConstPtr &amp;msg)
{
  ROS_INFO("vel %f", msg->twist.twist.linear.x);
  odom = *msg; // 追加
}
```

また、class rsj_robot_test_nodeの先頭に下記の変数定義を追加します。

```
class rsj_robot_test_node
{
private:
  nav_msgs::Odometry odom; // 追加
```

また、odomの中で方位を表す、クオータニオンをコンストラクタ(rsj_robot_test_node()関数)の最後で初期化しておきます。

```
rsj_robot_test_node():
{
  // (略)
  odom.pose.pose.orientation.w = 1.0; // 追加
}
```

メインループを以下のように変更してみましょう。

```
void mainloop()
{
  ROS_INFO("Hello ROS World!");

  ros::Rate rate(10.0);
  while(ros::ok())
  {
    ros::spinOnce();

    geometry_msgs::Twist cmd_vel;
    if(tf::getYaw(odom.pose.pose.orientation) > 1.57)
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
    }
    else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.1;
    }
    pub_twist.publish(cmd_vel);

    rate.sleep();
  }
}
```

これは、オドメトリのYaw角度(旋回角度)が1.57ラジアン(90度)を超えるまで、正方向に旋回する動作を表しています。

先ほどと同様にビルドし、ypspur_rosとrsj_robot_test_nodeを起動して動作を確認します。

### 小課題

1m前方に走行し、その後で帰ってくるコードを作成してみましょう。(1m前方に走行し180度旋回して1m前方に走行するか、もしくは、1m前方に走行し1m後方に走行すればよい。)

余裕があれば、四角形を描いて走行するコードを作成してみましょう。
