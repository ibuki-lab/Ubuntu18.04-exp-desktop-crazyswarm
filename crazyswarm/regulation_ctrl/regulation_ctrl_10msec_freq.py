#!/user/bin/env python
# coding: UTF-8
"""
    signal を使って一定周期で処理を実行するプログラムを作る。
    cfに一定周期で命令することは可能だが、実験終了後のデータの記録、cfの着陸が実行されない.
    解決策が見つからないのでsignal を使用した一定周期でのプログラム作成は見合わせる。
    
"""



import numpy as np
import time
import matplotlib.pyplot as plt
import pandas as pd
import datetime
import signal

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
class regulation_control( Frames_setup, Vel_controller):

    # このクラスの初期設定を行う関数
    def __init__(self, experiment_time, hight_start, pos_des):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__(pos_des)


    # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = hight_start
        self.pos_des = pos_des
        self.position_X = []
        self.position_Y = []
        self.position_Z = []
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
        self.child_frames = Frames_setup().children_frame
        # crazyflieの個体数も記録
        self.num_cf = len(self.child_frames) 
    
    # tfフレームの初期化，フレーム間変数用のリストの用意, フレーム間(原点と個々のcrazyflie)の位置関係が取得可能になる
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.frames = []

    def take_off(self):
        # まず離陸させる，高さはclass呼び出し時に設定
        self.allcfs.takeoff(targetHeight=self.hight_start, duration=2.0)
        rospy.sleep(5)  # 5秒静止

    def main1(self, signum, frame):
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
            
            # コントローラー(vel_controller.py)には現在の位置，姿勢(yaw角)，速度, と目標の位置, 姿勢(yaw角)，速度を渡して指令値を更新する.
            self.cmd_controller_output(pos_now=self.homogerous_matrixs[:3, 3], pos_des=np.array(self.pos_des), 
                                       yaw_now=self.RPY[2], yaw_des=0.0, dt=dt)
            
            # エージェントに速度指令を送る, Zはコンスタントに小さな値を入れておかないとまずいかも
            print(np.array([self.X_dot, self.Y_dot,self.Z_dot]), self.yaw_dot)
            cf.cmdVelocityWorld(np.array([self.X_dot, self.Y_dot, (0.01 + self.Z_dot)]), yawRate=self.yaw_dot)

            # 三次元位置を記録
            self.position_X.append(f.transform.translation.x)
            self.position_Y.append(f.transform.translation.y)
            self.position_Z.append(f.transform.translation.z)
            self.t.append(time.time())

    # 各エージェントに指令値を送る関数
    def main2(self):
        
        start_time = time.time() # 開始時刻の取得，実験時間を測るため

        signal.signal(signal.SIGALRM, self.main1)

        signal.setitimer(signal.ITIMER_REAL, 1, 0.05 * self.num_cf)

        try:
            while True:
                time.sleep(10000)
            
            
                # 実験時間が実験終了時間を過ぎたら機体を着陸させる
                if time.time() - start_time > self.exp_time:

                    # 速度指令を送ってた後に着陸指令を送るときはこの関数を個々のcrazyflieに対し呼び出す必要がある
                    for cf in self.allcfs.crazyflies:
                        cf.notifySetpointsStop(100)

                    # 着陸, targethightはプロペラ停止高度，durationは着陸にかける時間
                    self.allcfs.land(targetHeight=0.02, duration=4.0)
                    rospy.sleep(5) # 5秒停止
                    break
        
        except KeyboardInterrupt:
            print('_nCTRL-C pressed!')
            for cf in self.allcfs.crazyflies:
                cf.notifySetpointsStop(100)
            
            self.allcfs.land(targetHeight=0.02, duration=4)
            rospy.sleep(5)
            

        print("experiment finish!!")
        self.t = np.array(self.t) - start_time

        # 記録したデータ点をcsvファイルで保存する
        data = {"T": self.t, "X": self.position_X, "Y": self.position_Y, "Z": self.position_Z, 
                "desX": self.pos_des[0], "desY": self.pos_des[1], "desZ": self.pos_des[2],
                "Kxp": self.Xp,"Kxi": self.Xi,"Kxd": self.Xd,
                "Kyp": self.Yp,"Kyi": self.Yi,"Kyd": self.Yd,
                "Kzp": self.Zp,"Kzi": self.Zi,"Kzd": self.Zd}
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
    main_class = regulation_control(experiment_time, hight_start, position_destination)
    # 離陸
    main_class.take_off()
    # 実験
    main_class.main2()
    exit()