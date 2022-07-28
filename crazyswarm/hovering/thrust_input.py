#!/user/bin/env python
# coding: UTF-8

import numpy as np
import time
import matplotlib.pyplot as plt
import pandas as pd
import datetime

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
    def __init__(self, experiment_time, hight_start):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__()

    # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = hight_start

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


    # 各エージェントに指令値を送る関数
    def main(self):

        # 実験開始
        print("Experiment Start!!")
        time.sleep(1)
        start_time = time.time() # 開始時刻の取得，実験時間を測るため
        sampling_T = 0.005

        cf = self.allcfs.crazyflies[0]
        child_frame = self.child_frame

        zero = np.array([0.0, 0.0, 0.0])

        while True:
            t = time.time() - start_time
            # 原点と一つのcrazyflieとの座標変換を取得
            try:
                f = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))

            # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Error !')
                rospy.sleep(0.5)
                continue
            
            # 物理量設定
            self.Quaternion = (f.transform.rotation.x,f.transform.rotation.y,f.transform.rotation.z,f.transform.rotation.w)
            self.R = tf_conversions.transformations.quaternion_matrix(self.Quaternion)
            self.RPY = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)

            pwm = 0.1
            Thrust = np.array([0.0, 0.0, max(0.0, min(0.5, pwm))])
            cf.cmdFullState(pos=zero, vel=zero, acc=Thrust, yaw=0.0, omega=zero)


            # ループ時間設定
            self.interval(sampling_T, t)


            # 実験時間が実験終了時間を過ぎたら機体を着陸させる
            if time.time() - start_time > self.exp_time:

                # 速度指令を送ってた後に着陸指令を送るときはこの関数を個々のcrazyflieに対し呼び出す必要がある
                for cf in self.allcfs.crazyflies:
                    cf.notifySetpointsStop(100)

                # 着陸, targethightはプロペラ停止高度，durationは着陸にかける時間
                self.allcfs.land(targetHeight=0.02, duration=4.0)
                rospy.sleep(5) # 5秒停止
                break

        print("experiment finish!!")
        
    def interval(self, sample_T, t):
        if time.time() - t < sample_T:
            time.sleep(sample_T - (time.time() - t))



if __name__ == '__main__':
    # 初期位置の入力
    experiment_time = int(input("実験時間:"))
    hight_start = float(input("init_Z:"))
    # position_destination = list((float(input("dest_X:")), float(input("dest_Y:")), float(input("dest_Z:"))))
    
    # const_value_controlを初期化し，main関数を実行
    const_value_control(experiment_time, hight_start).main()
    exit()