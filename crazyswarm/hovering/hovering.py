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
        interval_start = time.time()
        sampling_T = 0.005

        cf = self.allcfs.crazyflies[0]
        child_frame = self.child_frame

        Zd = self.hight_start
        Perr_pre = 0
        Perr_sum = 0

        Kp = 300
        Kd = 50
        ki = 0.005
        Kangle = 65000
        while True:
                
            # 原点と一つのcrazyflieとの座標変換を取得
            try:
                f = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))

            # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Error !')
                rospy.sleep(0.5)
                continue

            interval_start = time.time()
            if time.time() - interval_start < sampling_T:
                time.sleep(sampling_T - (time.time() - interval_start))
            
            
            self.Quaternion = (f.transform.rotation.x,f.transform.rotation.y,f.transform.rotation.z,f.transform.rotation.w)
            self.R = tf_conversions.transformations.quaternion_matrix(self.Quaternion)
            self.RPY = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)

            Perr = Zd - f.transform.translation.z
            Derr = Perr - Perr_pre
            Perr_pre = Perr
            Perr_sum += Perr
            # rpy = - Kangle * np.matmul(np.transpose(self.R[:3, :3]), self.RPY) / 57.0
            Rd = (self.R[:3, :3] - np.transpose(self.R[:3, :3])) / 2.0
            rpy = -Kangle * np.array([Rd[2, 1], -Rd[0, 2], Rd[1, 0]])/57
            Thrust = np.array([0.0, 0.0, Kp * Perr + Kd * Derr + ki * Perr_sum])
            zero = np.array([0.0, 0.0, 0.0])
            print(Thrust, rpy)
            cf.cmdFullState(pos=zero, vel=zero, acc=Thrust, yaw=0.0, omega=rpy)

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

        # data = {"T": self.t, "X": self.position_X, "Y": self.position_Y, "Z": self.position_Z, 
        #         "desX": self.pos_des[0], "desY": self.pos_des[1], "desZ": self.pos_des[2],
        #         "Vxc": self.velocity_Xc}
        # # print(len(self.t), len(self.position_X), len(self.velocity_Xc))
        # data = {"T": self.t[:-1], "X": self.position_X, "Vc":self.velocity_Xc}

        # df = pd.DataFrame(data)
        # df.to_csv("regulation_ctl_{}".format(datetime.date.today()))
        



if __name__ == '__main__':
    # 初期位置の入力
    experiment_time = int(input("実験時間:"))
    hight_start = float(input("init_Z:"))
    # position_destination = list((float(input("dest_X:")), float(input("dest_Y:")), float(input("dest_Z:"))))
    
    # const_value_controlを初期化し，main関数を実行
    const_value_control(experiment_time, hight_start).main()
    exit()