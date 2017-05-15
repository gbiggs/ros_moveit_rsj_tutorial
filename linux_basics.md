---
title: Linuxの基本操作
date: 2017-05-09
---

基本的なLinuxの端末操作方法を学習します。

画面左のランチャーにある「端末」アイコンをクリックします。
![Terminal](images/terminal-icon.png)

ウインドウが開き、下記のような文字列が表示されます。
`ubuntu@ubuntu:~$ `
これは、__ユーザID__@__コンピュータ名__:__現在のディレクトリ__$」を表しており、__現在のディレクトリ__欄の「~」(チルダ)は、現在のユーザのホームディレクトリを表します。

## ディレクトリ操作 (ls, cd コマンド)

現在のディレクトリにある、ファイル、ディレクトリは、lsコマンドで表示できます。

    ubuntu@ubuntu:~$ ls
    Desktop  Documents  Downloads  Music  Pictures Public Templates Videos catkin_ws
    ubuntu@ubuntu:~$

現在のディレクトリは、cdコマンドで移動できます。以下のように、cdコマンド実行後、__現在のディレクトリ__欄が変化していることが確認できます。

    ubuntu@ubuntu:~$ cd catkin_ws
    ubuntu@ubuntu:~/catkin_ws$

「..」 は、一つ上のディレクトリを意味します。

    ubuntu@ubuntu:~/catkin_ws$ cd src/
    ubuntu@ubuntu:~/catkin_ws/src$ cd ..
    ubuntu@ubuntu:~/catkin_ws$

また、cdコマンドをディレクトリを指定せずに実行すると、ホームディレクトリに戻ります。

    ubuntu@ubuntu:~/catkin_ws$ cd
    ubuntu@ubuntu:~$

以降、セミナー中で入力するコマンドや実行結果は、ユーザID、コンピュータ名、ディレクトリを省略し、下記のように表記します。

    $ ls
    Desktop  Documents  Downloads  Music  Pictures Public Templates Videos catkin_ws
    $

## ディレクトリ作成 (mkdir コマンド)

指定した名前のディレクトリを作成します。

以下の例では、テンポラリディレクトリ(`/tmp/`)に、`test-directory`という名前のディレクトリを作成します。(テンポラリディレクトリは終了時にクリアされて空になります。)

    $ cd /tmp/
    $ ls
    ...
    $ mkdir test-directory
    $ ls
    ...  test-directory

## タブ補完

Linuxの端末でコマンドを入力する際、途中まで入力した後に __Tab__{: style="border: 1px solid black" } キーを押すと、可能な場合には、自動的に続きを入力してくれます。続きが出てこない場合は、もう少し先まで入力してから再度トライしましょう。

  $ cd ~/c [Tab]
  $ cd catkin_ws/

## 実行中のプログラムの停止

実行しているコマンドを途中で止めたい場合には、__Ctrl+c__{: style="border: 1px solid black" }を入力します。下記の例は、`yes`コマンド(`y`と表示し続けるコマンド)を実行して、__Ctrl+c__{: style="border: 1px solid black" }でそれを停止します。

    $ yes
    y
    y
    y
    y
    [Ctrl+c]
    ^C

    $

## コピー＆ペースト

端末の画面中でのコピー＆ペーストには、キー入力で行う方法と、マウスのみで行う方法があります。キー入力で行う際は、コピーしたい文字列を選択して__Ctrl+Shift+c__{: style="border: 1px solid black" }でコピー、__Ctrl+Shift+v__{: style="border: 1px solid black" }でペーストします。

  $ cd
  $ ls
  _Desktop_ Documents  Downloads  Music  Pictures Public Templates Videos catkin_ws
  [選択して Ctrl+Shift+c]
  $ cd [Ctrl+Shift+v]
  $ cd Desktop

なお、ブラウザなど、端末以外のソフトでは、__Ctrl+c__{: style="border: 1px solid black" }でコピーができます。

マウスのみでコピー＆ペーストを行う際は、コピーしたい文字列を選択して、そのまま中ボタンをクリックすることで、ペーストします。

    $ cd
    $ ls
    _Desktop_ Documents  Downloads  Music  Pictures Public Templates Videos catkin_ws
    $ cd [選択して中クリック]
    $ cd Desktop

## 端末を複数開く

ロボットのプログラムを実行する際、複数の端末ウインドウを使って操作する場合があります。__Ctrl+Shift+n__{: style="border: 1px solid black" }で新しいウインドウを、__Ctrl+Shift+t__{: style="border: 1px solid black" }で新しいタブを開くことができます。

## コマンドの履歴

端末のコマンド入力時に、上キーを押すと、これまでに入力したコマンドを再度呼び出すことができます。

    $ [上下キー]
