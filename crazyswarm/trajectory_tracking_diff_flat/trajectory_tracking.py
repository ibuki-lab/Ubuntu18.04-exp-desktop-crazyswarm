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
from crazyflie_driver.msg import GenericLogData
# from crazyflie_driver import FullState

# 関連モジュールのインポート
from traj_controller import Vel_controller
from tc_frames_setup import Frames_setup

# 定値制御
class trajectory_tracking( Frames_setup, Vel_controller):

    # このクラスの初期設定を行う関数
    def __init__(self, experiment_time, hight_start, c, r, T):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__(c, r, T)


    # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = hight_start
        self.t = []
        self.Px = []
        self.Py = []
        self.Pz = []
        self.Roll = []
        self.Pitch = []
        self.Yaw = []
        self.cmd_thrust_log = []
        self.cmd_roll_log = []
        self.cmd_pitch_log = []
        self.cmd_yaw_log = []
    
    # 軌道（円）のパラメータ設定
        self.T = T
        self.r = r
        self.Pd = 0
        self.Vd = 0
        self.Ad = 0
        self.yaw = 0
        self.wd = 0

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

        rospy.Subscriber('/cf14/log1', GenericLogData, self.cmd_log)
    
    def cmd_log(self, msg):
        self.cmd_thrust = msg.values[0]
        self.cmd_roll = msg.values[1]
        self.cmd_pitch = msg.values[2]
        self.cmd_yaw = msg.values[3]


    # 各エージェントに指令値を送る関数
    def main(self):
        
        # まず離陸させる，高さはclass呼び出し時に設定
        self.allcfs.takeoff(targetHeight=self.hight_start, duration=3.0)
        rospy.sleep(5)  # 5秒静止

        # 実験開始
        print("Experiment Start!!")
        start_time = time.time() # 開始時刻の取得，実験時間を測るため
        dt = 0.01
        interval_start = time.time()
        sampling_T = 0.01
        t = 0.0
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
                


                # クオータニオン(四元数)の取得
                self.Quaternion = (f.transform.rotation.x,f.transform.rotation.y,f.transform.rotation.z,f.transform.rotation.w)
                # オイラー角の取得
                self.RPY = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)
                # 回転行列の取得
                self.R = tf_conversions.transformations.quaternion_matrix(self.Quaternion)[:3, :3]
                
                # データ記録
                # self.t.append(t)
                # self.Px.append(f.transform.translation.x)
                # self.Py.append(f.transform.translation.y)
                # self.Pz.append(f.transform.translation.z)
                # self.Roll.append(self.RPY[0])
                # self.Pitch.append(self.RPY[1])
                # self.Yaw.append(self.RPY[2])
                self.cmd_thrust_log.append(self.cmd_thrust)
                self.cmd_roll_log.append(self.cmd_roll)
                self.cmd_pitch_log.append(self.cmd_pitch)
                self.cmd_yaw_log.append(self.cmd_yaw)
                # コントローラー(vel_controller.py)には現在の位置，姿勢(yaw角)，速度, と目標の位置, 姿勢(yaw角)，速度を渡して指令値を更新する.
                self.cmd_controller_output(R=self.R, t=t, T=self.T)
                # self.T = max(self.T - 0.003, 3)
                
                cf.cmdFullState(pos=self.Pd, vel=self.Vd, acc=self.Ad, yaw=self.yaw, omega=self.wd)


                # PIDの微分項用にサンプリング時間を設定，サンプリング時間は0.001秒*エージェントの数　+ 実行時間幅(ここでは切り捨ててる)

            if time.time() - interval_start < sampling_T:
                #print(sampling_T - (time.time() - interval_start))
                time.sleep(sampling_T - (time.time() - interval_start))

            # 実験時間が実験終了時間を過ぎたら機体を着陸させる
            if time.time() - start_time > self.exp_time:

                # 速度指令を送ってた後に着陸指令を送るときはこの関数を個々のcrazyflieに対し呼び出す必要がある
                for cf in self.allcfs.crazyflies:
                    cf.notifySetpointsStop(1)

                # 着陸, targethightはプロペラ停止高度，durationは着陸にかける時間
                self.allcfs.land(targetHeight=0.02, duration=4.0)
                rospy.sleep(5) # 5秒停止
                break
            t += interval_start - time.time()
        print("experiment finish!!")

        # 記録したデータ点をcsvファイルで保存する

        # data = {"T": self.t, "X": self.Px, "Y": self.Py, "Z": self.Pz, 
        #         "Roll": self.Roll, "Pitch": self.Pitch, "Yaw": self.Yaw}
        data = {"cmd_thrust":self.cmd_thrust_log, "cmd_roll":self.cmd_roll_log,
                "cmd_pitch":self.cmd_pitch_log, "cmd_yaw":self.cmd_yaw_log}
        df = pd.DataFrame(data)
        df.to_csv("trajectory_tracking2_{}_T{}_r{}".format(datetime.date.today(), self.T, self.r))

        # 記録したデータを描画
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
    # rospy.init_node('trajectory_tracking')
    experiment_time = int(input("実験時間:"))
    hight_start = float(input("init_Z:"))
    c = np.array([float(input("中心X:")), float(input("中心Y:")), hight_start])
    r = float(input("半径r:"))
    T = float(input("周期T:"))
    
    
    # const_value_controlを初期化し，main関数を実行
    trajectory_tracking(experiment_time, hight_start, c=c, r=r, T=T).main()
    exit()