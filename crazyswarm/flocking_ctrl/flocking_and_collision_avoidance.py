#!/user/bin/env python
# coding: UTF-8

import numpy as np
import time
import matplotlib.pyplot as plt
import pandas as pd
import datetime

from yaml.cyaml import CDumper

# from ros_ws.src.crazyswarm.scripts.pycrazyswarm.crazyswarm import Crazyswarm
from pycrazyswarm import *

import rospy
import tf2_ros
import tf_conversions
from tf_conversions import transformations
import tf

# 関連モジュールのインポート
from flock_controller_coll_avoid import Vel_controller
from flock_frames_setup import Frames_setup

# 群れ制御
class flocking_ctrl(Frames_setup, Vel_controller):
    
    # クラスの初期化
    def __init__(self, experiment_time):
        super(Frames_setup, self).__init__()

        # 座標系とエージェント数，フレームの設定
        self.world_frame = Frames_setup().world_frame
        self.child_frames = Frames_setup().children_frame
        self.num_cf = len(self.child_frames)
        self.cfs_state = []
        self.cfs_pos = np.array([0, 0, 0, 0])
        self.Quaternions = np.array([0, 0, 0, 0])

        # 実行時間，初期高度
        self.EXP_start = False
        self.EXP_end = False
        self.exp_time = experiment_time
        self.hight_start = 1.2 # 一定
        
        # self.hight_start = [abs(np.random.rand()) + 0.2 for _ in range(self.num_cf)] 　ランダム

        # rigid-body-network
        self.G = np.zeros((self.num_cf, self.num_cf))
        for i in range(1, self.num_cf-1):
            self.G[i, i-1] = 1; self.G[i, i+1] = 1
        self.G[0, i+1] = 1; self.G[0, 1] = 1
        self.G[i+1, 0] = 1; self.G[i+1, i] = 1 
        
        # self.G = np.array([[0.0, 0.0, 1.0, 1.0],
        #                    [0.0, 0.0, 1.0, 1.0],
        #                    [1.0, 1.0, 0.0, 0.0],
        #                    [1.0, 1.0, 0.0, 0.0]])
        
        #　crazyflieswarm関数の初期化
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs

        # tfフレームの初期化, フレーム間の位置関係の取得 1秒停止
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.X_P_regster = [ [ ] for _ in range(self.num_cf)]
        self.Y_P_regster = [ [ ] for _ in range(self.num_cf)]
        self.Z_P_regster = [ [ ] for _ in range(self.num_cf)]
        self.t_log = [0.0]
        time.sleep(1)


        # 各crazyflieの情報を取得できるか確認，
        self.g = []
        self.yaw = [0] * self.num_cf
        self.Q2M = transformations.quaternion_matrix
        self.Q2RPY = transformations.euler_from_quaternion
        for i, child_frame in zip(range(self.num_cf), self.child_frames):
            if i != 4:
                try:
                    # crazyflieのモーションキャプチャからの情報を取得
                    cf_state = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))
                    # 同次座標とヨー角を計算
                    Q = np.array([cf_state.transform.rotation.x, cf_state.transform.rotation.y, cf_state.transform.rotation.z, cf_state.transform.rotation.w])
                    g = self.Q2M(Q) + np.array([[0, 0, 0, cf_state.transform.translation.x],
                                                [0, 0, 0, cf_state.transform.translation.y],
                                                [0, 0, 0, cf_state.transform.translation.z],
                                                [0, 0, 0, 0]])
                    
                    self.g.append(g)    # 同次座標をリスト化
                    self.yaw.append(self.Q2RPY(Q)[2]) 
                except:
                    rospy.logerr('LookupTransform Error !')
                    rospy.sleep(0.5)
                    exit()

    # 同次座標を計算する関数
    def get_g(self, cf_state):
        # 姿勢あり
        '''
        Q = np.array([cf_state.transform.rotation.x, cf_state.transform.rotation.y, cf_state.transform.rotation.z, cf_state.transform.rotation.w])
        g = self.Q2M(Q) + np.array([[0, 0, 0, cf_state.transform.translation.x],
                                    [0, 0, 0, cf_state.transform.translation.y],
                                    [0, 0, 0, cf_state.transform.translation.z],
                                    [0, 0, 0, 0]])
        '''
        # yaw = self.Q2RPY(Q)[2]


        # 姿勢なし
        g = np.eye(4) + np.array([[0, 0, 0, cf_state.transform.translation.x],
                                    [0, 0, 0, cf_state.transform.translation.y],
                                    [0, 0, 0, cf_state.transform.translation.z],
                                    [0, 0, 0, 0]])
        yaw = 0
        return g, yaw
    def main(self):

        # 全てのcrazyflieを同じ高さまで上昇させる場合
        self.allcfs.takeoff(self.hight_start, 4)
        self.timeHelper.sleep(self.hight_start + 2.0)
        
        # それぞれ適当な高さまで上昇させる場合
        '''
        for i, cf in enumerate(self.allcfs.crazyflies):
            cf.takeoff(targetHeight=self.hight_start[i], duration=5)
            print("{}: start hight:{}".format(self.child_frames[i], self.hight_start[i]))
            self.timeHelper.sleep(1)
        '''

        print('Experiment Start!!!!')
        self.EXP_start = True
        start_time = time.time()
        pos_7 = np.array([0, 0, 0])        
        while True :
            t = time.time()

            for id, cf, child_frame in zip(range(self.num_cf), self.allcfs.crazyflies, self.child_frames):
                # ゲイン初期化
                Kxy = 0.1
                Kz = 0.001
                

                

                # 制御対象のcrazyflieの同次座標を更新
                cf_state = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))
                # 姿勢あり
                # self.g[id], self.yaw[id] = self.get_g(cf_state)
                if id != 6:
                    if id >= 7:
                        id = id - 1

                    self.g[id], yaw = self.get_g(cf_state)

                
                
                # 速度入力を決定
                    vel, yaw_rate = self.cmd_controller_output(i=id, G=self.G, g=self.g, yaw=self.yaw, pos_7=pos_7)



                # vel[0] += np.sin(2*np.pi*t/5) * 0.1
                # vel[1] += np.cos(2*np.pi*t/5) * 0.1

                # 実験範囲外に出ないようにする


                    xy = self.g[id][:2, 3]
                    if np.linalg.norm(xy, ord=2) > 1.8:
                        vel = self.safty_ctrl(cf_state_now=xy, vel=vel)
                        Kxy = 1

                    z = self.g[id][2, 3]
                    if z < 1.0 or z > 1.2:
                        vel = self.safty_ctrl_Z(cf_state_now=z, vel = vel)
                        Kz = 1
                    
                    # if id == 0:
                    #     vel[:2] += 3 * (xy_des - xy)

                    # ドローンの速度は限りなく小さくする
                    vel[:2] = vel[:2] * Kxy
                    vel[2] = vel[2] * Kz + 0.01

                    cf.cmdVelocityWorld(vel=vel, yawRate=0)
                    self.X_P_regster[id].append(xy[0])
                    self.Y_P_regster[id].append(xy[1])
                    self.Z_P_regster[id].append(z)
                else:
                    pos_7 = np.array([cf_state.transform.translation.x, cf_state.transform.translation.y, cf_state.transform.translation.z])
                    vel_7 = np.array([0.0, 0.0, 0.0])
                    if t - start_time > self.exp_time/4.0:
                        vel_7[0] = 0.05 * (1.9 - pos_7[0])
                        vel_7[1] = 0.2 * (0.0 - pos_7[1])
                        vel_7[2] = 0.3 * (1.05 - pos_7[2])
                        
                    else:
                        vel_7[2] = 0.3 * (1.05 - pos_7[2])
                    cf.cmdVelocityWorld(vel=vel_7, yawRate=0)

                    self.X_P_regster[6].append(pos_7[0])
                    self.Y_P_regster[6].append(pos_7[1])
                    self.Z_P_regster[6].append(pos_7[2])

            
                
                
            if time.time() - start_time > self.exp_time:
                self.EXP_end = False
                for cf in self.allcfs.crazyflies:
                    cf.notifySetpointsStop(100)
                self.allcfs.land(targetHeight=0.02, duration=5.0)
                rospy.sleep(5)
                print(time.time() - start_time - 5)
                break
            self.t_log.append(t - start_time)
        
        print('experiment finish')
        data = {}
        for i in range(self.num_cf):

            data["Xp_cf{}".format(i)] = self.X_P_regster[i]
            data["Yp_cf{}".format(i)] = self.Y_P_regster[i]
            data["Zp_cf{}".format(i)] = self.Z_P_regster[i]
        data["T"] = self.t_log
        now = datetime.datetime.now()
        df = pd.DataFrame(data)
        df.to_csv("flocking_ctl_{}_{}_{}_{}".format(now.month, now.day, now.hour, now.minute))

if __name__ == '__main__':
    experiment_time = int(input('実験時間:'))

    flocking_ctrl(experiment_time=experiment_time).main()





        

        
