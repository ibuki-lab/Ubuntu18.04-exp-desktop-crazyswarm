#!/user/bin/env python
# coding: UTF-8

import sys
import numpy as np
import time
import matplotlib.pyplot as plt
import pandas as pd
import datetime
import termios
from timeout_decorator import timeout, TimeoutError
# from ros_ws.src.crazyswarm.scripts.pycrazyswarm.crazyswarm import Crazyswarm
from pycrazyswarm import *

import rospy
import tf2_ros
import tf_conversions
import tf
# from crazyflie_driver import FullState

# 関連モジュールのインポート
from frames_setup import Frames_setup

# 定値制御
class const_value_control( Frames_setup):

    # このクラスの初期設定を行う関数
    def __init__(self):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__()

    # 入力水力記録用リストの用意
        self.pwm_register = []

    # crazyswamの関数を固定
        # Crazyswarm()の呼び出し
        self.swarm = Crazyswarm()
        # TimeHelperの呼び出し
        self.timeHelper = self.swarm.timeHelper
        # すべてのcrazyflieの情報を取得
        self.allcfs = self.swarm.allcfs
        
    # 座標系とエージェントの数を設定, frames_setup.pyで取得した各剛体のフレーム名を取得
        self.world_frame = Frames_setup().world_frame
        self.child_frame = Frames_setup().children_frame[0]
    
    # tfフレームの初期化，フレーム間変数用のリストの用意, フレーム間(原点と個々のcrazyflie)の位置関係が取得可能になる
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.frames = []
        time.sleep(0.5)

        

    # 各エージェントに指令値を送る関数
    def main(self):

        # 実験開始
        print("Experiment Start!!")
        time.sleep(1)
        input_thrust = 0.0
        cf = self.allcfs.crazyflies[0]
        child_frame = self.child_frame
        
        # 入力リスト
        input_thrust = [0.0, 2500, 5000, 7500, 10000, 12500, 15000, 17500, 20000, 22500, 25000]
        input_thrust = input_thrust + sorted(input_thrust, revase=True)
        for input in input_thrust:
            Tzero = time.time() 
            while True:
                Ts = time.time() - Tzero
                # 原点と一つのcrazyflieとの座標変換を取得
                try:
                    f = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))

                # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr('LookupTransform Error !')
                    rospy.sleep(0.5)
                    continue
                
                print(input_thrust)            
                termios.tcsetattr(self.fd, termios.TCSANOW, self.old)
                zero = np.array([0.0, 0.0, 0.0])

                cf.cmdFullState(pos=zero, vel=zero, acc=np.array([0.0, 0.0, input]), yaw=0.0, omega=zero)
                self.pwm_register.append(input_thrust*1000.0)
                
                if Ts > 10.0:
                    break
        print("experiment finish!!")
        data = {"input thrust": self.pwm_register}
        df = pd.DataFrame(data)
        df.to_csv("thrust_data_log{}".format(datetime.date.today()))

if __name__ == '__main__':
    # const_value_controlを初期化し，main関数を実行
    const_value_control().main()
    exit()