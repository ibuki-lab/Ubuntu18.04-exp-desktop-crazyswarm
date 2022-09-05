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
        data_t = pd.read_csv('data_t.csv')
        self.data_t = np.array(data_t)

    

    # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = hight_start
        self.pos_des = pos_des
        self.position_X = []
        self.position_Y = []
        self.position_Z = []
        self.velocity_Xc = []
        self.velocity_Xc_learn = []
        self.X_dot_log = []
        self.Vdot = []
        self.Vdot_sl = []
        self.t = []
        self.deltas = []

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
        rospy.sleep(6)  # 6秒静止

        # 実験開始
        print("Experiment Start!!")
        start_time = time.time() # 開始時刻の取得，実験時間を測るため
        t = 0
        dt = 0.01
        interval_start = time.time()
        sampling_T = 0.01

        X_prev = 0
        self.t.append(0.0)
        v_pre = 0
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
                interval_start = time.time()
                if time.time() - interval_start < sampling_T:
                    time.sleep(sampling_T - (time.time() - interval_start))
                new_t = time.time() - start_time
                
                # クオータニオン(四元数)の取得
                #self.Quaternion = (f.transform.rotation.x,f.transform.rotation.y,f.transform.rotation.z,f.transform.rotation.w)
                # オイラー角の取得
                #self.RPY = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)
                # 同次座標の取得
                #self.homogerous_matrixs = tf_conversions.transformations.quaternion_matrix(self.Quaternion) + np.array([[0, 0, 0, f.transform.translation.x],
                                                                                                                        # [0, 0, 0, f.transform.translation.y],
                                                                                                                        # [0, 0, 0, f.transform.translation.z],
                                                                                                                        # [0, 0, 0, 0]])
                X = f.transform.translation.x; Y = f.transform.translation.y; Z = f.transform.translation.z
                state = np.array([X, Y, Z])
                
                # コントローラー(vel_controller.py)には現在の位置，姿勢(yaw角)，速度, と目標の位置, 姿勢(yaw角)，速度を渡して指令値を更新する.
                self.cmd_controller_output(pos_now=state, pos_des=np.array(self.pos_des), 
                                           yaw_now=0, yaw_des=0.0, dt=new_t - self.t[-1])
                
                # 微分で速度計算
                # v_now = (X - X_prev)/(new_t - self.t[-1])
                # X_dot = 0.9 * v_pre + 0.1 * v_now
                # new_X = np.array([X, X_dot])
                # # 次回計算用vとｘを記録
                # v_pre = v_now
                # X_prev = X

                delta = 0
                
                # # plot gauss regulation
                # for n in range(len(self.data_c)):
                #     # delta = c(i)          * e^{-||dataX(i) - newdataX(i)||^2}
                #     delta += self.data_c[n] * np.exp(-10 * (Z - self.data_x[n])**2)
                # print("delta:{}".format(delta))
                
                # plot poly regulation
                # for n in range(len(self.data_c)):
                #     # delta = c(i)          * e^{-||dataX(i) - newdataX(i)||^2}
                #     delta += self.data_c[n] * (1 + Z * self.data_x[n] + (Z * self.data_x[n])**2 + (Z * self.data_x[n])**3 + (Z * self.data_x[n])**4 + (Z * self.data_x[n])**5 + (Z * self.data_x[n])**6 + (Z * self.data_x[n])**7 + (Z * self.data_x[n])**8)
                # print("delta:{}".format(delta))
                
                # if Z > 1.5:
                #     delta = 0
                
                # # 誤差項を補正する
                # self.Z_dot = 0.4*(1.5 - Z) - delta

                cf.cmdVelocityWorld(np.array([self.X_dot, self.Y_dot, self.Z_dot]), yawRate=self.yaw_dot)
                # if time.time() - start_time > 3.0:
                #     cf.cmdVelocityWorld(np.array([0.4, 0.0, 0.01]), 0.0)
                #     self.velocity_Xc.append(0.4)
                # else:
                #     cf.cmdVelocityWorld(np.array([0.0, 0.0, 0.01]), 0.0)
                #     self.velocity_Xc.append(0.0)
                # 三次元位置を記録
                self.position_X.append(X)
                self.position_Y.append(Y)
                # self.position_Z.append(Z)
                # self.velocity_Xc.append(self.Z_dot)
                self.t.append(new_t)
                # self.Vdot.append(v_now)
                # self.Vdot_sl.append(X_dot)
                # self.deltas.append(delta)
                # print(f.transform.translation.z)
                # PIDの微分項用にサンプリング時間を設定，サンプリング時間は0.001秒*エージェントの数　+ 実行時間幅(ここでは切り捨ててる)



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
        data = {"T": self.t[1:], "X": self.position_Z, "Vc":self.velocity_Xc, "X_dot":self.Vdot, "X_dot_sl":self.Vdot_sl, "delta":self.deltas}

        df = pd.DataFrame(data)
        df.to_csv("regulation_ctl_{}".format(datetime.date.today()))

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
    position_destination = list((float(input("dest_X:")), float(input("dest_Y:")), float(input("dest_Z:"))))
    
    # const_value_controlを初期化し，main関数を実行
    const_value_control(experiment_time, hight_start, position_destination).main()
    exit()