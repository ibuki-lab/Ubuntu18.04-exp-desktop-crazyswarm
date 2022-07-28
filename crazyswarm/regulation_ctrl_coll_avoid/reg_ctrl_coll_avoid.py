#!/user/bin/env python
# coding: UTF-8

import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import datetime

# from ros_ws.src.crazyswarm.scripts.pycrazyswarm.crazyswarm import Crazyswarm
from pycrazyswarm import *
import rospy
import tf2_ros
import tf_conversions
import tf
# from crazyflie_driver import FullState

# 自作モジュールのインポート
from vel_controller_collision_avoidance import vel_controller_coll_avoid
from frames_setup import Frames_setup

# 定置制御での二次計画法による衝突回避
class const_value_control( Frames_setup, vel_controller_coll_avoid):
    
# このクラスの初期設定を行う関数
    def __init__(self, experiment_time, hight_start, pos_des):
    # 速度コントローラーのクラスを継承
        super(Frames_setup, self).__init__(pos_des = pos_des, alpha = 1)


    # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = hight_start
        self.pos_des = pos_des
        self.pos_obs = np.array([0.0, 0.0])
        self.r = 0.3
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
        self.num_cf = len(self.child_frames) 
    
    # tfフレームの初期化
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        
# 各エージェントに指令値を送る関数
    def main(self):
        
    # 離陸させる，高さはclass呼び出し時に設定
        self.allcfs.takeoff(targetHeight=self.hight_start, duration=2.0)
        rospy.sleep(5)  # 5秒静止

    # 実験開始
        print("Experiment Start!!")
        start_time = time.time()
        t = 0
        dt = 0.001
        while True:

        # 各エージェントに指令, !!!!!!!!!! 子フレームと命令対象のエージェントが一致している事が絶対条件 !!!!!!!!
            for cf, child_frame in zip(self.allcfs.crazyflies, self.child_frames):
                
            # 2フレーム間の座標変換を取得
                try:
                    f = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))

            # 取得できなかった場合は１秒間処理を停止し再び再開する
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr('LookupTransform Error !')
                    rospy.sleep(1.0)
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
                
            # コントローラー(vel_controller.py)には現在の位置，姿勢(yaw角)，速度 と目標の位置, 姿勢(yaw角)，速度 さらに障害物の位置と半径 を渡して指令値を更新する.
                self.cmd_controller_collision_avoidance_output(pos_now=self.homogerous_matrixs[:3, 3], pos_des=np.array(self.pos_des), 
                                           yaw_now=self.RPY[2], yaw_des=0.0, pos_obs=self.pos_obs, r=self.r, dt=dt)
            
            # エージェントに速度指令を送る
                print(np.array([self.X_dot, self.Y_dot, self.Z_dot]), self.yaw_dot)
                cf.cmdVelocityWorld(np.array([self.X_dot, self.Y_dot, (0.01 + self.Z_dot)]), yawRate=self.yaw_dot)

            # 三次元位置を記録
                self.position_X.append(f.transform.translation.x)
                self.position_Y.append(f.transform.translation.y)
                self.position_Z.append(f.transform.translation.z)
                self.t.append(t)

                # サンプリング時間は0.001秒*エージェントの数　+ 実行時間幅(切り捨てるか迷い)
                rospy.sleep(dt)
                t += dt
        # 実行時間終了で機体を着陸
            if time.time() - start_time > self.exp_time:
                for cf in self.allcfs.crazyflies:
                    cf.notifySetpointsStop(100)
                # 着陸, targethightはプロペラ停止高度，durationは着陸にかける時間
                self.allcfs.land(targetHeight=0.02, duration=4.0)
                rospy.sleep(5)
                break

        print("experiment finish!!")

        # 記録したデータ点をcsvファイルで保存する
        data = {"T": self.t, "X": self.position_X, "Y": self.position_Y, "Z": self.position_Z, 
                "desX": self.pos_des[0], "desY": self.pos_des[1], "desZ": self.pos_des[2],
                "Kxp": self.Xp,"Kxi": self.Xi,"Kxd": self.Xd,
                "Kyp": self.Yp,"Kyi": self.Yi,"Kyd": self.Yd,
                "Kzp": self.Zp,"Kzi": self.Zi,"Kzd": self.Zd,
                "pos_obs_x": self.pos_obs[0], "pos_obs_y": self.pos_obs[1], "r": self.r}
        df = pd.DataFrame(data)
        df.to_csv("reg_ctl_coll_avoid_{}".format(datetime.date.today()))

        # 記録したデータの描画
        fig = plt.figure()
        circle = plt.Circle(self.pos_obs, self.r, fill=False)

        ax1 = fig.add_subplot(1, 1, 1)
        ax1.plot(self.position_X, self.position_Y)
        ax1.add_artist(circle)
        ax1.set_xlabel('X[m]')
        ax1.set_ylabel('Y[m]')
        ax1.set_xlim(-1.5, 1.5)
        ax1.set_ylim(-1.5, 1.5)
        plt.grid()
        plt.show()
        
        

if __name__ == '__main__':
    # 初期位置の入力
    experiment_time = int(input("実験時間:"))
    hight_start = float(input("初期高度:"))
    position_destination = list((float(input("X:")), float(input("Y:")), float(input("Z:"))))

    const_value_control(experiment_time, hight_start, position_destination).main()







                











                
            

            





