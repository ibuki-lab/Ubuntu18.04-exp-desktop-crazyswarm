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
from tf_conversions import transformations
import tf

# 関連モジュールのインポート
from drones_coll_avoid_controller import Vel_controller
from drones_coll_avoid_frames_setup import Frames_setup

# 高度の合意制御
class drones_coll_avoid(Frames_setup, Vel_controller):

    # このクラスの初期設定を行う関数
    def __init__(self, experiment_time, alpha):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__(alpha)

#########################################################################################
        
        # 座標系とエージェントの数を設定, frames_setup.pyで取得した各剛体のフレーム名を取得
        self.world_frame = Frames_setup().world_frame
        self.child_frames = Frames_setup().children_frame
        self.num_cf = len(self.child_frames)
        self.cfs_state = []
        self.cfs_pos = np.array([0, 0, 0, 0])
        self.Quaternions = np.array([0, 0, 0, 0])
        
#######################################################################################
        
        # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = 1.0 # 1メートル
        self.des2 = np.array([[-1.0, 0.0, self.hight_start],
                             [1.0, 0.0, self.hight_start]])

#######################################################################################

        # crazyswamの関数を固定
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs

##########################################################################################

        # tfフレームの初期化，フレーム間(原点と個々のcrazyflie)の位置関係が取得可能になる
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        
        # tfフレームが取得可能か確認する
        self.g = []
        self.yaw = [0] * self.num_cf
        self.Q2M = transformations.quaternion_matrix
        self.Q2RPY = transformations.euler_from_quaternion
        time.sleep(2)
        for i, child_frame in enumerate(self.child_frames):
            # 各crazyflieの状態を一つのリストにまとめる
            try:
                # crazyflieのモーションキャプチャからの情報を取得
                cf_state = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))
                # 同次座標とヨー角を計算
                # Q = np.array([cf_state.transform.rotation.x, cf_state.transform.rotation.y, cf_state.transform.rotation.z, cf_state.transform.rotation.w])
                g = np.eye(4) + np.array([[0, 0, 0, cf_state.transform.translation.x],
                                            [0, 0, 0, cf_state.transform.translation.y],
                                            [0, 0, 0, cf_state.transform.translation.z],
                                            [0, 0, 0, 0]])

                self.g.append(g)    # 同次座標をリスト化
                # self.yaw.append(self.Q2RPY(Q)[2]) 

            # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Error !')
                rospy.sleep(0.5)
                exit()
        
##########################################################################################

    # 同次座標を計算する関数
    def get_g(self, cf_state):
        # Q = np.array([cf_state.transform.rotation.x, cf_state.transform.rotation.y, cf_state.transform.rotation.z, cf_state.transform.rotation.w])
        g = np.eye(4) + np.array([[0, 0, 0, cf_state.transform.translation.x],
                                    [0, 0, 0, cf_state.transform.translation.y],
                                    [0, 0, 0, cf_state.transform.translation.z],
                                    [0, 0, 0, 0]])
        
        # yaw = self.Q2RPY(Q)[2]
        
        return g

    # 各エージェントに指令値を送る関数
    def main(self):

        # すべてのドローンを設定した高さまで上昇させる
        self.allcfs.takeoff(targetHeight=self.hight_start, duration=1.0+self.hight_start)
        self.timeHelper.sleep(self.hight_start + 2.0)

        # 実験開始, 微小時間の初期化
        print("Experiment Start!!")
        start_time = time.time() # 開始時刻の取得，実験時間を図るため

        time_interval = 0.01
        

        while True:
            # 各crazyflieに指令を送る, !!!!!!!!!! 子フレーム(cfx)と命令対象のcrazyflieが一致している事が絶対条件 !!!!!!!!
            for id, cf, child_frame in zip(range(self.num_cf), self.allcfs.crazyflies, self.child_frames):
                
                s_time = time.time()

                # 制御対称のcrazyflieの状態を取得
                cf_state = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))
                self.g[id] = self.get_g(cf_state)
                
                self.cmd_controller_output(i = id,
                                        des = self.des2[id, :],
                                        g = self.g
                                        )

                # crazyflieに速度指令を送る
                self.vel[:2] = self.vel[:2]*0.1
                self.vel[2] += 0.01
                cf.cmdVelocityWorld(vel=self.vel, yawRate=0)

                if time.time() - s_time < time_interval:
                    time.sleep(time_interval - (time.time() - s_time))
    
    
                # 実験時間を過ぎたら機体を着陸させる
            
            if time.time() - start_time > self.exp_time:
                for cf in self.allcfs.crazyflies:
                    cf.notifySetpointsStop(100)
                self.allcfs.land(targetHeight=0.02, 
                            duration=4.0)
                rospy.sleep(5) # 5秒停止
                print(time.time() - start_time)
                break
            
        print("experiment finish!!")

if __name__ == '__main__':
    # 初期位置の入力
    experiment_time = int(input("実験時間:"))
    
    # const_value_controlを初期化し，main関数を実行
    drones_coll_avoid(experiment_time, 20).main()