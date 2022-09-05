#!/user/bin/env python
# coding: UTF-8

from re import T
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
from tc_controller import Vel_controller
from tc_frames_setup import Frames_setup

# 追従性御　八の字
class trajectory_controll(Frames_setup, Vel_controller):

    # このクラスの初期設定を行う関数
    def __init__(self, experiment_time, hight_start, trajectory_gain):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__(trajectory_gain)


    # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = hight_start
        self.position_X_desired = []
        self.position_Y_desired = []
        self.position_X = []
        self.position_Y = []
        self.position_Z = []
        self.Vxc = []
        self.Vyc = []
        self.Vdot = []
        self.Vdot_sl = []
        self.t = []
        self.deltas = []
    
    # ベクトルCを取得
        data_c = pd.read_csv('data_c.csv')
        self.data_c = np.array(data_c).ravel()
        # 訓練データX1を取得
        data_x = pd.read_csv('data_x.csv')
        self.data_x = np.array(data_x)
        #訓練データX2を取得
        data_t = pd.read_csv('data_t.csv')
        self.data_t = np.array(data_t)

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
        self.num_cf = len(self.child_frames) 
    
    # tfフレームの初期化，フレーム間(原点と個々のcrazyflie)の位置関係が取得可能になる
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
    
    # 各エージェントに指令値を送る関数
    def main(self):

        # まず離陸させる，高さはclass呼び出し時に設定
        self.allcfs.takeoff(targetHeight=self.hight_start, duration=2.0)
        time.sleep(2.0)
        # 初期位置まで移動 8字の中心を原点に設定しているから
        self.allcfs.crazyflies[0].goTo(np.array([0.0, 0.0, 0.0]), yaw=0.0, duration=5.0)
        print(len(self.allcfs.crazyflies))
        time.sleep(2.0)
        # 実験開始, 微小時間の初期化
        print("Experiment Start!!")
        start_time = time.time() # 開始時刻の取得，実験時間を図るため
        dt = 0.01
        T = 5
        pre_X = 0.0
        Xd_pre = 0.0
        while True:
            ts = time.time()
            t = time.time() - start_time

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
                
            # クオータニオン(四元数)の取得
                self.Quaternion = (f.transform.rotation.x,f.transform.rotation.y,f.transform.rotation.z,f.transform.rotation.w)
            # オイラー角の取得
                self.RPY = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)
            # 同次座標の取得
                self.homogerous_matrixs = tf_conversions.transformations.quaternion_matrix(self.Quaternion) + np.array([[0, 0, 0, f.transform.translation.x],
                                                                                                                        [0, 0, 0, f.transform.translation.y],
                                                                                                                        [0, 0, 0, f.transform.translation.z],
                                                                                                                        [0, 0, 0, 0]])
                X = f.transform.translation.x; Y = f.transform.translation.y; Z = f.transform.translation.z


            # 追従する軌道の計算, 1周期10秒
                A = 1.0 # 振幅
                W = 2 * np.pi / T
                X_desired = A*np.sin(W*t)
                Xv_tra = A*W*np.cos(W*t)
                Y_desired = 0*np.sin(W*t)
                Yv_tra = 0*W*np.cos(W*t)
                Z_desired = self.homogerous_matrixs[3, 3] # 二次元平面上の軌道追従を考えるから高さはそのまま

            # 目標位置，姿勢の決定
                pos_desired = np.array([X_desired, Y_desired, Z_desired])
                yaw_desired = 0.0
                vel_tra = (Xv_tra, Yv_tra)

            # 速度計算
                Xd = (self.homogerous_matrixs[0, 3] - pre_X)/dt
                pre_X = self.homogerous_matrixs[0, 3]
                Xdot_pre = 0.9 * Xd_pre + 0.1 * Xd
                Xd_pre = Xd
            # コントローラーに現在の位置，姿勢，目標の位置，姿勢，追従速度を渡す 速度指令値の更新
                self.cmd_controller_output(pos_now=self.homogerous_matrixs[:3, 3], yaw_now=self.RPY[2], pos_des=pos_desired, yaw_des=yaw_desired, vel_tra=vel_tra)
            
            # 補正項計算
                delta = 0
                
                # plot kernel tracking
                dataX = np.array([1, X, t, X_desired])
                for n in range(len(self.data_c)):
                    # delta = c(i)          * e^{-||dataX(i) - newdataX(i)||^2}
                    delta += self.data_c[n] * np.matmul(dataX, np.array([1, self.data_x[n], self.data_t[n], np.sin(W * self.data_t[n])]))
                print("delta:{}".format(delta))
                
                # plot multiple tracking
                # for n in range(len(self.data_c)):
                    # delta = c(i)          * e^{-||dataX(i) - newdataX(i)||^2}
                # delta = 0.02659 + 0.55268 * X + -0.002917 * t
                # print("delta:{}".format(delta))
                
                if t < 1:
                    delta = 0

                self.X_dot = 0.4 * (X_desired - f.transform.translation.x) + Xv_tra - delta
                cf.cmdVelocityWorld(np.array([self.X_dot, self.Y_dot, self.Z_dot]), yawRate=self.yaw_dot)
                print(self.X_dot, self.Y_dot, self.Z_dot, self.yaw_dot)

        
            # 目標位置と実際の位置を記録
                self.position_X.append(self.homogerous_matrixs[0, 3])
                self.position_Y.append(self.homogerous_matrixs[1, 3])
                self.position_X_desired.append(X_desired)
                self.position_Y_desired.append(Y_desired)
                self.Vxc.append(self.X_dot)
                self.Vyc.append(self.Y_dot)
                self.Vdot.append(Xd)
                self.Vdot_sl.append(Xdot_pre)
                self.t.append(t)
                self.deltas.append(delta)
        
            # ループ周期を一定にする
            if time.time() - ts < dt:
                time.sleep(dt - (time.time() - ts))


        # 実験時間を過ぎたら機体を着陸させる
            if time.time() - start_time > self.exp_time:
                for cf in self.allcfs.crazyflies:
                    cf.notifySetpointsStop(100)
                self.allcfs.land(targetHeight=0.02, duration=4.0)
                rospy.sleep(5) # 5秒停止
                print(time.time() - start_time)
                break
            
        print("experiment finish!!")

        # データの保存
        data = {"T": self.t, "X": self.position_X,
                "X_des": self.position_X_desired, 
                "Vcx": self.Vxc, "Xdot":self.Vdot, "Xdot_dl":self.Vdot_sl
                }
        df = pd.DataFrame(data)
        df.to_csv("trajectory_ctrl_X{}".format(datetime.date.today()))

        # 理想的な起動と実際の軌跡

        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(self.position_X, self.position_Y, label="Gain:{}".format(self.trajectory_gain))
        ax.plot(self.position_X_desired, self.position_Y_desired)
        ax.set_xlabel('X[m]')
        ax.set_ylabel('Y[m]')
        ax.legend()
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        plt.grid()
        plt.show()


if __name__ == '__main__':
    # 初期位置の入力
    experiment_time = int(input("実験時間:"))
    hight_start = float(input("初期高度:"))
    trajectory_gain = float(input("trajecoty_gain:"))
    
    # const_value_controlを初期化し，main関数を実行
    trajectory_controll(experiment_time, hight_start, trajectory_gain).main()