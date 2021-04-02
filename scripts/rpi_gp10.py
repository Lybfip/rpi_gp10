#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import smbus            # I2C制御用
import RPi.GPIO as GPIO # GPIO制御用
from std_msgs.msg import Bool
from std_msgs.msg import UInt8

# グローバル変数 ---------------------------------------------------------------------------------
STB  = 14               # STB出力 GPIO14(JP7:Default) / GPIO12(JP8)
TRG  = 15               # TRG入力 GPIO15(JP5:Default) / GPIO13(JP6)
trg_last = 1            # TRG 入力初期値
in_last = 0             # IN 端子入力初期値

pub_inputSignal = None  # 入力信号トピックパブリッシャ

#RPi-GP10用インターフェースの初期設定(※下記設定は変更しないでください)
def init_GP10():
    global i2c
    global i2c_adrs
    global STB
    global TRG
    global node_name

    GPIO.setmode(GPIO.BCM)                              # GPIO番号で指定
    GPIO.setup(27, GPIO.OUT, initial=GPIO.HIGH )        # RPi-GP10絶縁回路用電源ON
    rospy.sleep(0.5)                                    # 電源安定待ち
    GPIO.setup(STB, GPIO.OUT, initial=GPIO.LOW )        # STB端子出力設定 HIGH (オープンコレクタ)
    GPIO.setup(TRG, GPIO.IN, pull_up_down=GPIO.PUD_OFF) # TRG端子入力設定

    try:
        i2c.write_byte_data(i2c_adrs, 0x06, 0x00)           # 出力端子(ポート0)方向設定 出力
        i2c.write_byte_data(i2c_adrs, 0x04, 0x00)           # 出力端子(ポート0)極性設定 反転なし
        i2c.write_byte_data(i2c_adrs, 0x07, 0xFF)           # 入力端子(ポート1)方向設定 入力
        i2c.write_byte_data(i2c_adrs, 0x05, 0xFF)           # 入力端子(ポート1)極性設定 反転あり
        rospy.sleep(0.1)
    except:
        print "[{}] RPi-GP10 の初期化に失敗しました。".format(node_name)
        GPIO.output(27,False)                           # RPi-GP10絶縁電源OFF
        GPIO.cleanup()
        sys.exit()

# 終了させるための関数
def exit():
    # ここを実行すると出力が OFF してしまう。
    GPIO.output(27, False)   # RPi-GP10の絶縁回路用電源OFF
    GPIO.cleanup()
    sys.exit()

# 入力信号の監視
def monitorTheInputSignal():
    global i2c
    global i2c_adrs
    global pub_inputSignal
    global pub_trg
    global trg_last
    global in_last

    # ポーリングフラグで処理を分岐する
    if f_polling:
        # IN 端子のデータをパブリッシュ
        in_last = i2c.read_byte_data(i2c_adrs, 0x01)
        pub_inputSignal.publish(in_last)
        # TRG 端子のデータをパブリッシュ
        trg_last = GPIO.input(TRG)
        pub_trg.publish(trg_last)
    else:
        # IN 端子データが変化していたらパブリッシュする
        in_now = i2c.read_byte_data(i2c_adrs, 0x01)
        if in_now != in_last:
            pub_inputSignal.publish(in_now)
            in_last = in_now
        # トリガ入力が変化したら、パブリッシュする。
        trg_now = GPIO.input(TRG)
        if trg_now != trg_last:
            pub_trg.publish(trg_now)
            trg_last = trg_now

# 出力データトピックサブスクライバコールバック
def cb_output(data):
    global i2c
    global i2c_adrs
    # 出力端子にトピックデータを出力
    i2c.write_byte_data(i2c_adrs, 0x02, data.data)

# ストローブトピックサブスクライバコールバック関数
def cb_stb(data):
    global STB
    # ストローブ端子にトピックデータを出力
    GPIO.output(STB, data.data)

# ポーリングトピックサブスクライバコールバック関数
def cb_polling(data):
    global f_polling
    f_polling = data.data

# メイン関数 ----------------------------------------------------------------------------
if __name__ == "__main__":

    i2c_adrs = 0x20                 # TCA9535 I2Cアドレス 0x20 (RA1～RA6)

    # ノードの初期化
    node_name = 'rpi-gp10'          # ノード名デフォルト値
    rospy.init_node(node_name)
    node_name = rospy.get_name()

    # rosparam の読み込み
    RATE = rospy.get_param(node_name + '/rate', default=10)
    initialOutput = rospy.get_param(node_name + '/initOut', default=0)
    initialStb = rospy.get_param(node_name + '/initStb', default=0)
    f_polling = rospy.get_param(node_name + '/polling', default=True)
    
    # i2c の初期化
    try:
        i2c  = smbus.SMBus(1)   # RPi-GP10 は i2c-1 を使用
    except Exception as e:
        print "[{}] i2cで例外： {}".format(node_name, e)
        sys.exit()
    
    # RPi-GP10初期化
    init_GP10()         

    i2c.write_byte_data(i2c_adrs, 0x02, initialOutput)  # 出力端子に初期値を出力
    GPIO.output(STB, initialStb)                        # STB端子に初期値を出力

    pub_inputSignal = rospy.Publisher(node_name + '/input', UInt8, queue_size=1)    # パブリッシャ：入力信号
    pub_trg = rospy.Publisher(node_name + '/trg', UInt8, queue_size=1)              # パブリッシャ：トリガ出力
    rospy.Subscriber(node_name + '/output', UInt8, cb_output)                       # サブククライバ：出力信号
    rospy.Subscriber(node_name + '/stb', UInt8, cb_stb)                             # サブスクライバ：ストローブ出力
    rospy.Subscriber(node_name + '/polling', Bool, cb_polling)                      # サブスクライバ：ポーリング

    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        # 入力信号のポーリング
        monitorTheInputSignal()
        rate.sleep()

    # 終了処理
    exit()
