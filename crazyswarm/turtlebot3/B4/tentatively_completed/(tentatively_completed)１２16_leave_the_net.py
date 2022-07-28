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

# 関連モジュールのインポート
from ln_cc_controller import Vel_controller
from ln_cc_frames_setup import Frames_setup

# 高度の合意制御
class formation_ctrl(Vel_controller, Frames_setup):

    # このクラスの初期設定を行う関数
    def __init__(self, experiment_time):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__()
        
        # twist = Twistみたいに関数を呼びだすときの変数にしている
        self.V = Vel_controller()
#########################################################################################
        
        # 座標系とエージェントの数を設定, frames_setup.pyで取得した各剛体のフレーム名を取得
        self.world_frame = Frames_setup().world_frame
        self.child_frames = Frames_setup().children_frame
        self.turtle_frame = Frames_setup().turtle_frame
        # lenで台数の数を確認している(3台であったら、3と決まる)
        self.num_cf = len(self.child_frames)
        self.cfs_state = np.array([0, 0, 0, 0.0])
        
        
#######################################################################################
        
        # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = 1.0
        self.X_log = [[] for _ in range(self.num_cf)]
        self.Y_log = [[] for _ in range(self.num_cf)]
        self.t_log = [0.0]

#######################################################################################

        # 隣接行列，次数行列，グラフラプラシアンの生成
        # グラフラプラシアン = 次数行列 - 隣接行列
        # np.eye(3)は3*3の単位行列
        # np.ones(2, 3)は2*3のすべての要素が1の行列
        regulation_matrix = np.array([[0.0, 1.0, 0.0, 0.0],
                                      [-1.0, 0.0, 1.0, 0.0],
                                      [-1.0, 0.0, 0.0, 1.0],
                                      [2.0, -1.0, -1.0, -1.0]])
        self.L = (self.num_cf + 1) * np.eye(self.num_cf + 1) - np.ones((self.num_cf + 1, self.num_cf + 1)) - regulation_matrix
        
        
########################################################################################       

        # crazyswamの関数を固定
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs

##########################################################################################

        # tfフレームの初期化，フレーム間(原点と個々のcrazyflie)の位置関係が取得可能になる
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        # tfフレームが取得可能か確認する
        time.sleep(2)
        for i, child_frame in enumerate(self.child_frames):
            # 各crazyflieの状態を一つのリストにまとめる
            try:
                # crazyflieはVのスタックを作りたいがためにinitでもlookupしている
                cur_cf = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))
                self.cfs_state = np.vstack((self.cfs_state, np.array([cur_cf.transform.translation.x, cur_cf.transform.translation.y, cur_cf.transform.translation.z, 0.0])))
            # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Error !')
                rospy.sleep(0.5)
                exit()
        
        # 不要な行を取り除く
        # self.cfs_state = self.cfs_state[1:, :]

##########################################################################################

    # 各エージェントに指令値を送る関数
    def main(self):

        # 全てのエージェントを一定の高さまで上昇させる
        self.allcfs.takeoff(self.hight_start, 4)
        time.sleep(4)
        # 実験開始, 微小時間の初期化
        print("Experiment Start!!")
        start_time = time.time() # 開始時刻の取得，実験時間を図るため
        while True:
            # 各crazyflieに指令を送る, !!!!!!!!!! 子フレーム(cfx)と命令対象のcrazyflieが一致している事が絶対条件 !!!!!!!!
            for cf, child_frame in zip(self.allcfs.crazyflies, self.child_frames):
                # iは0からスタートしている
                i = self.child_frames.index(child_frame)
                # 制御対称のcrazyflieの状態を取得
                cur_cf = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))
                t = self.tfBuffer.lookup_transform(self.world_frame, self.turtle_frame, rospy.Time(0))
                # 姿勢，位置の情報を更新
                self.cfs_state[i, :] = np.array([cur_cf.transform.translation.x, cur_cf.transform.translation.y, cur_cf.transform.translation.z,0.0])                    # 位置姿勢をまとめる
                self.cfs_state[3, :] = np.array([t.transform.translation.x, t.transform.translation.z, 0.0, 0.0])
                
                # turtlebot3における変換
                # クオータニオン(四元数)の取得
                self.Quaternion_t = (t.transform.rotation.x,t.transform.rotation.z,-t.transform.rotation.y,t.transform.rotation.w)
                # オイラー角の取得
                self.RPY_t = tf_conversions.transformations.euler_from_quaternion(self.Quaternion_t)  
                self.turtle_state_X = t.transform.translation.x
                self.turtle_state_Y = t.transform.translation.z

                
                # コントローラーに全てのcfの位置情報yaw角の情報を渡す
                vel = self.V.cmd_controller_output(cfs_state_now=self.cfs_state,
                                     yaw_des=0.0,
                                     L=self.L,
                                     cur_cf_idx=i,
                                     turtle_pos_X=self.turtle_state_X,
                                     turtle_pos_Y=self.turtle_state_Y)
                # 衝突回避
                vel = self.V.collision_avoidance(cfs_state_now=self.cfs_state,
                                                vel=vel,
                                                cur_cf_idx=i)
                # 網に接触しないようにする
                cf_xy_state = self.cfs_state[i, :2]
                if np.linalg.norm(cf_xy_state, ord=2) > 1.5:
                    vel = self.V.safty_ctrl(cf_state_now=cf_xy_state, vel=vel)


                # crazyflieに速度指令を送る
                cf.cmdVelocityWorld(vel=vel[:3], 
                                yawRate=vel[3])
                self.X_log[i].append(self.cfs_state[i, 0])
                self.Y_log[i].append(self.cfs_state[i, 1])
                
            # 実験時間を過ぎたら機体を着陸させる
            
            if time.time() - start_time > self.exp_time:
                for cf in self.allcfs.crazyflies:
                    cf.notifySetpointsStop(100)
                self.allcfs.land(targetHeight=0.02, 
                            duration=4.0)
                rospy.sleep(5) # 5秒停止
                print(time.time() - start_time)
                break
            self.t_log.append(time.time() - start_time)
            
        print("experiment finish!!")

        # データ記録
        data = {}
        for i in range(self.num_cf):
            data['cf{}x'.format(i+1)] = self.X_log[i]
            data['cf{}y'.format(i+1)] = self.Y_log[i]
        data['T'] = self.t_log

        now = datetime.datetime.now()
        df = pd.DataFrame(data)
        df.to_csv("data{}_{}_{}_{}".format(now.month, now.day, now.hour, now.minute))

if __name__ == '__main__':
    # 初期位置の入力
    experiment_time = int(input("実験時間:"))
    
    # const_value_controlを初期化し，main関数を実行
    formation_ctrl(experiment_time).main()