# rpi_gp10

RPi-GP10用ROSノード

このソフトウェアは、ラトックシステム株式会社製のI2C絶縁型デジタル入出力ボード「RPi-GP10」用のROSノードです。
Raspberry Pi 3B+, 4 上の ROS Melodic で動作を確認しています。

1. インストール

msbusをインストールしてください。

~$ sudo apt-get install python-smbus

RPi.GPIOをインストールしてください。

~$ sudo apt-get install python-rpi.gpio

このリポジトリをForkして、~/catkin_ws/src/ の下にクローンしてください。

~$ cd ~/catkin_ws/src

~/catkin_ws/src$ git clone git@github.com:<ユーザー名>/rpi_gp10.git

ビルドしてください。

~/catkin_ws/src$ cd ~/catkin_ws

~/catkin_ws$ catkin_make

I2Cのデバイスファイルに書き込み権限がないとエラーになります。
私は、~/.bashrc ファイルの末尾に、

sudo chmod 666 /dev/i2c-1

を追記しました。

2. 動作確認

次のコマンドでROSノードが起動します。

~$ roslaunch rpi-gp10 rpi-gp10.launch

別のターミナルを起動し、rostopic コマンドで動作を確認できます。
IN0-7の値の表示：

~$ rostopic echo /rpi_gp10/input

トリガ入力の表示：

~$ rostopic echo /rpi_gp10/trg

OUT0 = L, OUT1-7 = H の出力

~$ rostopic pub -1 /rpi_gp10/output std_msgs/UInt8 0x01

ストローブ出力を L にする。

~$ rostopic pub -1 /rpi_gp10/output std_msgs/UInt8 1

3. launch ファイルの param の説明

  rate: 入力信号ポーリングのレート。単位は Hz
  initOut: OUT 端子の初期値。0 で OUT 0-7 = H 出力
  initStb: STB 端子の初期値。0 で STB = H 出力

以上です。
