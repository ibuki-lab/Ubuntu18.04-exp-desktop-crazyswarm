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
        self.cfs_state = np.array([0, 0, 0, 0])
        self.Quaternions = np.array([0, 0, 0, 0])
        
#######################################################################################
        
        # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = 0.5 # 1メートル
        self.centers = []
        self.r = [0.5 + np.random.rand()*0.5 for _ in range(self.num_cf)] # 各ドローンの描く円の半径を設定
        self.T = [16 + np.random.randn() for _ in range(self.num_cf)] # 各ドローンの描く円の周期
        self.position_X = [[] for _ in range(self.num_cf)]
        self.position_Y = [[] for _ in range(self.num_cf)]
        self.t = []

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
        time.sleep(2)
        for i, child_frame in enumerate(self.child_frames):
            # 各crazyflieの状態を一つのリストにまとめる
            try:

                '''
                cfs_state
                [[cf1_x, cf1_y, cf1_z, cf1_yaw],
                 [cf2_x, cf2_y, cf2_z, cf2_yaw],
                              :
                              :
                              :
                 [cfn_x, cfn_y, cfn_z, cfn_yaw]]
                
                cfs_Quaternions
                [[cf1_x, cf1_y, cf1_z, cf1_w],
                 [cf2_x, cf2_y, cf2_z, cf2_w],
                              :
                              :
                              :
                 [cfn_x, cfn_y, cfn_z, cfn_w]]
                '''
                cur_cf = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))
                self.Quaternions = np.vstack((self.Quaternions, np.array([cur_cf.transform.rotation.x,cur_cf.transform.rotation.y,cur_cf.transform.rotation.z,cur_cf.transform.rotation.w])))
                self.cfs_state = np.vstack((self.cfs_state, np.array([cur_cf.transform.translation.x, cur_cf.transform.translation.y, cur_cf.transform.translation.z, 0.0])))
                print("半径:{}  周期:{}".format(self.r[i], self.T[i]))
                # 各ドローンが描く円の中心座標を設定(今回は初期位置-(0.5, 0.0))
                self.centers.append(np.array([cur_cf.transform.translation.x, cur_cf.transform.translation.y]) - np.array([0.4, 0.0]))
            # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Error !')
                rospy.sleep(0.5)
                exit()
        
        # 不要な行を取り除く
        self.Quaternions = self.Quaternions[1:, :]
        self.cfs_state = self.cfs_state[1:, :]

##########################################################################################


    # 各エージェントに指令値を送る関数
    def main(self):

        # すべてのドローンを設定した高さまで上昇させる
        self.allcfs.takeoff(targetHeight=self.hight_start, duration=1.0+self.hight_start)
        self.timeHelper.sleep(self.hight_start + 2.0)

        # 実験開始, 微小時間の初期化
        print("Experiment Start!!")
        start_time = time.time() # 開始時刻の取得，実験時間を図るため
        dt = 0.001
        t = 0
        print(self.centers)
        while True:
            # 各crazyflieに指令を送る, !!!!!!!!!! 子フレーム(cfx)と命令対象のcrazyflieが一致している事が絶対条件 !!!!!!!!
            for cf, child_frame, circle_center, r, T in zip(self.allcfs.crazyflies, self.child_frames, self.centers, self.r, self.T):
                i = self.child_frames.index(child_frame)
                # 制御対称のcrazyflieの状態を取得
                cur_cf = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))
                # 姿勢，位置の情報を更新
                self.Quaternions[i, :] = np.array([cur_cf.transform.rotation.x,cur_cf.transform.rotation.y,cur_cf.transform.rotation.z,cur_cf.transform.rotation.w])            # クオータニオンの取得
                self.RPY = tf_conversions.transformations.euler_from_quaternion(self.Quaternions[i, :])                                                                         # オイラー角の取得
                self.cfs_state[i, :] = np.array([cur_cf.transform.translation.x, cur_cf.transform.translation.y, cur_cf.transform.translation.z, self.RPY[2]])                    # 位置姿勢をまとめる


            
                # 実験開始からの経過時間と前回の計算からの微小時間を計算を取得
                t = time.time() - start_time
                # 制御対象のcfの描く円の軌道を取得
                omega = (2*np.pi/T)
                position_desired = np.array([circle_center[0] + r*np.cos(omega*t), circle_center[1] + r*np.sin(omega*t), 1.0])
                vel_des = np.array([-r*omega*np.sin(omega*t),  r*omega*np.cos(omega*t), .0, .0])
                # コントローラーに全てのcfの位置情報yaw角の情報を渡す
                self.cmd_controller_output(cfs_pos_now=self.cfs_state,
                                     pos_des = position_desired,
                                     vel_des=vel_des,
                                     yaw_des=0.0,
                                     cur_cf_idx=i
                                     )
                # crazyflieに速度指令を送る
                cf.cmdVelocityWorld(vel=self.vel[:3], 
                                yawRate=self.vel[3])
                
                # サンプリング時間
                rospy.sleep(dt)
                t += dt
            
                # # 各cfの高度を記録を記録
                # self.position_X[i].append(self.cfs_pos[i, 0])
                # self.position_Y[i].append(self.cfs_pos[i, 1])
                self.t.append(t)

            # 実験時間を過ぎたら機体を着陸させる
            
            if time.time() - start_time > self.exp_time:
                for cf in self.allcfs.crazyflies:
                    cf.notifySetpointsStop(100)
                self.allcfs.land(targetHeight=0.02, 
                            duration=4.0)
                rospy.sleep(5) # 5秒停止
                print(time.time() - start_time)
                print(self.t[300:500])
                break
            
        print("experiment finish!!")
        # # データの保存
        # max_len = len(self.position_Z[0]) - 2
        # print(len(self.t), len(self.position_Z[0]), len(self.position_Z[1]))
        # data = {"T": self.t[:max_len]}
        # for i in range(self.num_cf):
        #     data.update({"Z{}".format(i): self.position_Z[i][:max_len]})
        # print(data)
        # df = pd.DataFrame(data)
        # df.to_csv("consensus_ctrl_{}".format(datetime.date.today()))

        # # 理想的な起動と実際の軌跡

        # fig = plt.figure()
        # ax = fig.add_subplot(1, 1, 1)
        # for i in range(self.num_cf):
        #     ax.plot(self.position_Z[i][:max_len], label="agent-{}".format(i+1))
        # ax.set_xlabel('T[s]')
        # ax.set_ylabel('Z[m]')
        # ax.legend()
        # ax.set_ylim(0, 1.5)
        # plt.grid()
        # plt.show()

if __name__ == '__main__':
    # 初期位置の入力
    experiment_time = int(input("実験時間:"))
    
    # const_value_controlを初期化し，main関数を実行
    drones_coll_avoid(experiment_time, 20).main()