#!/user/bin/env python
# coding: UTF-8


from lib2to3.pytree import type_repr
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
from vel_controller import Vel_controller
from frames_setup import Frames_setup

# 定値制御
class const_value_control( Frames_setup, Vel_controller):

    # このクラスの初期設定を行う関数
    def __init__(self, experiment_time, hight_start, pos_des):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__(pos_des)

    # ガウスカーネル　訓練データ、カーネル行列、
        # ベクトルCを取得
        data_c = pd.read_csv('data_c.csv')
        self.data_c = np.array(data_c).ravel()
        # 訓練データX1を取得
        data_x = pd.read_csv('data_x.csv')
        self.data_x = np.array(data_x)
        #訓練データX2を取得
        # data_xdot = pd.read_csv('data_xdot.csv')
        # data_xdot = np.array(data_xdot)

        # # 訓練データを結合
        # self.data_X = np.hstack((data_x, data_xdot))
        # self.data_X = data_x


    # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = hight_start
        self.pos_des = pos_des
        self.position_X = []
        self.position_Y = []
        self.position_Z = []
        self.velocity_Zc = []
        self.velocity_Xc_learn = []
        self.X_dot_log = []
        self.t = []
        self.delta = []

    # crazyswamの関数を固定
        # Crazyswarm()の呼び出し
        self.swarm = Crazyswarm()
        # TimeHelperの呼び出し
        self.timeHelper = self.swarm.timeHelper
        # すべてのcrazyflieの情報を取得
        self.allcfs = self.swarm.allcfs
        
    # 座標系とエージェントの数を設定, frames_setup.pyで取得した各剛体のフレーム名を取得
        self.world_frame = Frames_setup().world_frame
        self.child_frames = Frames_setup().children_frame
        # crazyflieの個体数も記録
        self.num_cf = len(self.child_frames) 
    
    # tfフレームの初期化，フレーム間変数用のリストの用意, フレーム間(原点と個々のcrazyflie)の位置関係が取得可能になる
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.frames = []


    # 各エージェントに指令値を送る関数
    def main(self):
        
        # まず離陸させる，高さはclass呼び出し時に設定
        self.allcfs.takeoff(targetHeight=self.hight_start, duration=5.0)
        rospy.sleep(5)  # 5秒静止

        # 実験開始
        print("Experiment Start!!")
        start_time = time.time() # 開始時刻の取得，実験時間を測るため
        interval_start = time.time()
        sampling_T = 0.01

        X_prev = 0
        kp = 0.4
        Zpre = 1.0
        t_pre = 0.0
        dt = 1
        delta = 0
        flag = False
        while True:
            # 各crazyflieに指令を送る, !!!!!!!!!! 子フレーム(cf?)と命令対象のcrazyflieが一致している事が絶対条件 !!!!!!!!
            for cf, child_frame in zip(self.allcfs.crazyflies, self.child_frames):
                
                # 原点と一つのcrazyflieとの座標変換を取得
                try:
                    f = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))

                # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr('LookupTransform Error !')
                    rospy.sleep(0.5)
                    continue              
                
                if time.time() - interval_start < sampling_T:
                    time.sleep(sampling_T - (time.time() - interval_start))
                interval_start = time.time()
                
                # 位置取得
                X = f.transform.translation.x; Y = f.transform.translation.y; Z = f.transform.translation.z
                ZD = (Z - Zpre)/dt
                Zpre = Z
                # state = np.array([X, Y, Z])
                if Z < 1.0 or flag:
                    flag = True
                    # 着陸制御則
                    Zd = 1.0 - 0.1 * t
                    Z_dot = - 0.1
                    # if t > 9.64:
                    #     Z_dot = -0.0
                    #     Zd = 0.0
                    #     input_Z = kp*(0.036-Z)
                    if Z < 0.036:
                        input_Z = -5
                    else:
                        input_Z = Z_dot + kp * (Zd - Z)

                        

                        if Z < 100:
                        # # 学習後用 カーネル
                            new_X = np.array([1, np.log(Z-0.035)**3, np.log(1.01 - Z)**3])
                            # 誤差デルタを計算
                            delta = 0
                            for n in range(len(self.data_c)):
                                # delta = c(i)          * e^{-||dataX(i) - newdataX(i)||^2}
                                # print(self.data_X[n, :])
                                # print(self.data_X[n, :] - new_X)
                                # ガウスカーネル
                                # delta += self.data_c[n] * 15.60580919 * np.exp(-(np.linalg.norm(self.data_X[n, :] - new_X, ord=2)**2)/(1.397429139))
                                # 対数カーネル
                                #delta += self.data_c[n] * np.dot(new_X, np.array([1, self.data_x[n], np.log(self.data_x[n]/4.0)]))
                                # delta += self.data_c[n] * np.dot(new_X, np.array([1, np.log(self.data_x[n])**3]))
                                delta += self.data_c[n] * np.dot(new_X, np.array([1, np.log(self.data_x[n]-0.035)**3,  np.log(1.01 - self.data_x[n])**3]))


                            # # 誤差項を補正する
                            input_Z = input_Z - delta
                    
                    # # 学習御用　重回帰
                    # if Z < 1.0:
                    #     delta = 0.0077*Z + 1.2079*ZD + 0.1076
                    #     input_Z = input_Z - delta
                    #     self.delta.append(delta)

                    cf.cmdVelocityWorld(np.array([0.0, 0.0, input_Z]), 0.0)
                    self.velocity_Zc.append(input_Z)
                    self.position_Z.append(Z)
                    self.t.append(t)
                    t = time.time() - Ts
                    # self.delta.append(delta)
                    # self.velocity_Zc_learn.append(self.X_dot)
                    # t = time.time() - start_time
                    # dt = t - t_pre
                    # t_pre = t
                else:
                    cf.cmdVelocityWorld(np.array([0.0, 0.0, -0.01]), 0.0)
                    Ts = time.time()
                    t = 0


            # 実験時間が実験終了時間を過ぎたら機体を着陸させる
            if time.time() - start_time > self.exp_time:
                cf.cmdVelocityWorld(np.array([0.0, 0.0, 0.0]), 0.0)

                # 速度指令を送ってた後に着陸指令を送るときはこの関数を個々のcrazyflieに対し呼び出す必要がある
                for cf in self.allcfs.crazyflies:
                    cf.notifySetpointsStop(100)

                # # 着陸, targethightはプロペラ停止高度，durationは着陸にかける時間
                # self.allcfs.land(targetHeight=0.02, duration=4.0)
                # rospy.sleep(5) # 5秒停止
                break

        print("experiment finish!!")

        # 記録したデータ点をcsvファイルで保存する
        # data = {"T": self.t, "X": self.position_X, "Y": self.position_Y, "Z": self.position_Z, 
        #         "desX": self.pos_des[0], "desY": self.pos_des[1], "desZ": self.pos_des[2],
        #         "Kxp": self.Xp,"Kxi": self.Xi,"Kxd": self.Xd,
        #         "Kyp": self.Yp,"Kyi": self.Yi,"Kyd": self.Yd,
        #         "Kzp": self.Zp,"Kzi": self.Zi,"Kzd": self.Zd,
        #         "Vxc": self.velocity_Xc}
        # data = {"T": self.t, "X": self.position_X, "Y": self.position_Y, "Z": self.position_Z, 
        #         "desX": self.pos_des[0], "desY": self.pos_des[1], "desZ": self.pos_des[2],
        #         "Vxc": self.velocity_Xc}
        # # print(len(self.t), len(self.position_X), len(self.velocity_Xc))
        data = {"T": self.t, "X": self.position_Z, "Vc":self.velocity_Zc}
        # data = {"T": self.t[:-1], "X": self.position_Z, "Vc":self.velocity_Zc}

        df = pd.DataFrame(data)
        df.to_csv("ML_landdata_after_Z_Zdot{}".format(datetime.date.today()))

        # # 記録したデータを描画
        # fig = plt.figure()

        # ax1 = fig.add_subplot(1, 2, 1)
        # ax1.plot(self.position_X, self.position_Y)
        # ax1.set_xlabel('X[m]')
        # ax1.set_ylabel('Y[m]')
        # ax1.set_xlim(-1.5, 1.5)
        # ax1.set_ylim(-1.5, 1.5)
        # plt.grid()

        # ax2 = fig.add_subplot(1, 2, 2)
        # ax2.plot(self.t, self.position_Z)
        # ax2.set_ylabel('Z[m]')
        # ax2.set_xlabel('t[s]')
        # ax2.set_xlim(0, 1.5)
        # plt.show()
        



if __name__ == '__main__':
    # 初期位置の入力
    experiment_time = int(input("実験時間:"))
    hight_start = float(input("init_Z:"))
    position_destination = list((float(input("dest_X:")), float(input("dest_Y:")), float(input("dest_Z:"))))
    
    # const_value_controlを初期化し，main関数を実行
    const_value_control(experiment_time, hight_start, position_destination).main()
    exit()