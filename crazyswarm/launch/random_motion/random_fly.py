#!/user/bin/env python
# coding: UTF-8

import numpy as np
import time
import matplotlib.pyplot as plt
import pandas as pd
import datetime
from scipy.signal import max_len_seq

# from ros_ws.src.crazyswarm.scripts.pycrazyswarm.crazyswarm import Crazyswarm
from pycrazyswarm import *

import rospy
import tf2_ros
import tf_conversions
import tf
# from crazyflie_driver import FullState

# 関連モジュールのインポート
from random_fly_controller import Random_fly_controller
from frames_setup import Frames_setup

# 定値制御
class random_fly( Frames_setup, Random_fly_controller):

    # このクラスの初期設定を行う関数
    def __init__(self, experiment_time, hight_start):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__()


    # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = hight_start
        self.position_X = []
        self.position_Y = []
        self.position_Z = []
        self.input_X = []
        self.input_Z = []
        self.input_Y = []
        self.t = []

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
        self.cf = self.allcfs.crazyflies[0]

    
    # tfフレームの初期化，フレーム間変数用のリストの用意, フレーム間(原点と個々のcrazyflie)の位置関係が取得可能になる
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.frames = []

    # self.a1 = np.random.rand()
    # self.a2 = np.random.rand()
    # self.a3 = np.random.rand()
    # self.a4 = np.random.rand()


    # self.b1 = np.random.rand()
    # self.b2 = np.random.rand()
    # self.b3 = np.random.rand()
    # self.b4 = np.random.rand()
    
    # self.omega = 2 * np.pi / 10

    # m系列信号を生成
    def gen_mls(self, iniM, iniN):
        n = len(iniN)
        M = iniM + [0] * (2**n - 1 - len(iniM))
        for i in range(n, len(M)):
            M[i] = self.cal_M(i, n-1, M, iniN, M[i-n]*iniN[-n])
        return M
    def cal_M(self, i, n, M, h, tmp):
        if n == 1:
            return (tmp + M[i-n]*h[-n])%2
        else:
            return self.cal_M(i, n-1, M, h, (tmp + M[i-n]*h[-n])%2)
    
    


    # 各エージェントに指令値を送る関数
    def main(self):
        
        # まず離陸させる，高さはclass呼び出し時に設定
        self.allcfs.takeoff(targetHeight=self.hight_start, duration=2.0)
        rospy.sleep(5)  # 5秒静止

        # 実験開始
        print("Experiment Start!!")
        start_time = time.time() # 開始時刻の取得，実験時間を測るため

        n = 11
        m_signal_x = self.gen_mls([1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1], [0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1])
        m_signal_y = self.gen_mls([0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0])
        m_signal_z = self.gen_mls([1, 1, 0, 1, 0, 1, 0, 1, 0, 0 ,1, 0, 1, 1, 1, 1], [1, 0, 0, 1, 1, 0, 0, 1, 1, 0 ,1, 0, 1, 0, 1, 1])

        sampling_T = 0.01
        Time_input = 0.05
        Time_input_s = 0
        while True:
            
            for signal_x, signal_y, signal_z in zip(m_signal_x, m_signal_y, m_signal_z):
                
                interval_start = time.time()


                # 原点と一つのcrazyflieとの座標変換を取得
                try:
                    f = self.tfBuffer.lookup_transform(self.world_frame, self.child_frame, rospy.Time(0))

                # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr('LookupTransform Error !')
                    rospy.sleep(0.5)
                    continue              
                
                # クオータニオン(四元数)の取得
                self.Quaternion = (f.transform.rotation.x,f.transform.rotation.y,f.transform.rotation.z,f.transform.rotation.w)
                # オイラー角の取得
                self.RPY = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)
                # 同次座標の取得
                self.homogerous_matrixs = tf_conversions.transformations.quaternion_matrix(self.Quaternion) + np.array([[0, 0, 0, f.transform.translation.x],
                                                                                                                        [0, 0, 0, f.transform.translation.y],
                                                                                                                        [0, 0, 0, f.transform.translation.z],
                                                                                                                        [0, 0, 0, 0]])


                # self.cmd_controller_output(pos_now=self.homogerous_matrixs[:3, 3], pos_des=[0, 0, 0.5], 
                #                            yaw_now=self.RPY[2], yaw_des=0.0)
                
                # t = time.time()
                # self.X_dot = self.a1 * np.sin(self.omega * t) + self.a2 * (np.sin(self.omega * t)**2) + self.
                
                
                
                
                # m系列信号を用いてx軸方向とy軸方向の速度入力を決定する.
                if Time_input < time.time() - Time_input_s:
                    if -1.5 < self.homogerous_matrixs[0, 3] < 1.5:

                        self.X_dot = 0.6*signal_x - 0.3

                    if -1.5 < self.homogerous_matrixs[1, 3] < 1.5:

                        self.Y_dot = 0.6*signal_y - 0.3
                    if 0.3 < self.homogerous_matrixs[2, 3] < 2.3:

                        self.Z_dot = 0.6*signal_z - 0.3
                    Time_input_s = time.time()

                    self.cf.cmdVelocityWorld(np.array([self.X_dot, self.Y_dot, (0.01 + self.Z_dot)]), yawRate=self.yaw_dot)
                # エージェントに速度指令を送る, Zはコンスタントに小さな値を入れておかないとまずいかも
                    print(np.array([self.X_dot, self.Y_dot,self.Z_dot]), self.yaw_dot)

                    # 三次元位置を記録
                    self.position_X.append(f.transform.translation.x)
                    self.position_Y.append(f.transform.translation.y)
                    self.position_Z.append(f.transform.translation.z)
                    self.input_X.append(self.X_dot)
                    self.input_Y.append(self.Y_dot)
                    self.input_Z.append(self.Z_dot)
                    self.t.append(time.time() - start_time)

                if time.time() - interval_start < sampling_T:
                    time.sleep(sampling_T - (time.time() - interval_start))

                if time.time() - start_time > self.exp_time:
                    break

            
            # 実験時間が実験終了時間を過ぎたら機体を着陸させる
            if time.time() - start_time > self.exp_time:

                # 速度指令を送ってた後に着陸指令を送るときはこの関数を個々のcrazyflieに対し呼び出す必要がある
                self.cf.notifySetpointsStop(100)

                # 着陸, targethightはプロペラ停止高度，durationは着陸にかける時間
                self.allcfs.land(targetHeight=0.02, duration=4.0)
                rospy.sleep(5) # 5秒停止
                break

        print("experiment finish!!")

        # 記録したデータ点をcsvファイルで保存する
        data = {"T": self.t, "X": self.position_X, "Y": self.position_Y, "Z": self.position_Z,
                "X_dot": self.input_X, "Y_dot": self.input_Y, "Z_dot": self.input_Z}
        df = pd.DataFrame(data)
        df.to_csv("random_fly_data{}3".format(datetime.date.today()))

        # 記録したデータを描画
        fig = plt.figure()

        ax1 = fig.add_subplot(1, 2, 1)
        ax1.plot(self.position_X, self.position_Y)
        ax1.set_xlabel('X[m]')
        ax1.set_ylabel('Y[m]')
        ax1.set_xlim(-1.5, 1.5)
        ax1.set_ylim(-1.5, 1.5)
        plt.grid()

        ax2 = fig.add_subplot(1, 2, 2)
        ax2.plot(self.t, self.position_Z)
        ax2.set_ylabel('Z[m]')
        ax2.set_xlabel('t[s]')
        ax2.set_xlim(0, 1.5)
        plt.show()
        

if __name__ == '__main__':
    # 初期位置の入力
    experiment_time = int(input("実験時間:"))
    hight_start = float(input("init_Z:"))

    
    # const_value_controlを初期化し，main関数を実行
    random_fly(experiment_time, hight_start).main()
    exit()