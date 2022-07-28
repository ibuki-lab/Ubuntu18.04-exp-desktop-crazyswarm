# !/user/bin/env python
# coding: UTF-8

# すべてを選択して「Ctrl + /」でコメントアウトできる
# 行いたい入力としては、「並進運動」
import numpy as np
# numpyはPythonの一般的な科学計算パッケージ
# センサデータを用いて作業しているのであれば、データの配列を処理するために用いるとよい
# import ライブラリ名 as 好きな名前(今回はnpと省略して記述するために「as」を用いている)
import time
import matplotlib.pyplot as plt

# crazyflie関連のライブラリ
# from ros_ws.src.crazyswarm.scripts.pycrazyswarm.crazyswarm import Crazyswarm
from pycrazyswarm import *

import rospy
import tf2_ros
import tf_conversions
# 上の二つはcrazyflieに用いている座標変換のライブラリ
import tf
# turtlebot3のtf変換で用いる
# from crazyflie_driver import FullState
from math import cos, sin
from geometry_msgs.msg import Twist, PoseStamped

BUGER_MAX_LIN_LEVEL = 0.22
BUGER_MAX_ANG_LEVEL = 2.84

L = 0.1 # [m]
kp = 0.2
twist = Twist

# 関連モジュールのインポート
from tc_vel_controller import Vel_controller
from tc_frames_setup import Frames_setup

# 定値制御
# Pythonでクラスを定義する際のキーワード(予約語)
# classは関数の寄せ集めのようなもの
# 別のファイルのclassの変数なども共有できる
# class このファイルにおけるclassの名前(他のファイルにおけるclassの名前)
# 変数の前に「self」とつけると、別の違う関数でも共通の変数として用いることができる
class const_value_control(Frames_setup, Vel_controller):

    # このクラスの初期設定を行う関数
    # __init__はclassを呼び出した際に、一番初めに行って欲しいこと
    # この場合は、値の初期化をしたいがために__init__を用いている
    # pos_desと省略されているのは、三番目の引数という位置で判断しているため
    # def __init__(self, experiment_time, hight_start, pos_des):
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
        self.turtle_frame = Frames_setup().turtle_frame
        # crazyflieの個体数も記録
        self.num_cf = len(self.child_frames) 
     
    ###
    # tfフレームの初期化，フレーム間(原点と個々のcrazyflie)の位置関係が取得可能になる
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
    
    # turtlebot3の定置制御
    # def callback(self, data):
    #     global twist
    #     t = tf.transformations.euler_from_quaternion((data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))
    #     self.Xp = data.pose.position.x
    #     self.Yp = data.pose.position.y
    #     self.Zp = data.pose.position.z
    #     theta = t[2]
    #     vec = np.array([0], [0], [0])
    #     Xc = self.Xp + L * cos(theta)
    #     Yc = self.Yp + L * sin(theta)
    #     Vtil = (vec[0, 0] - Xc)
    #     Wtil = (vec[1, 0] - Yc)
    #     V = Vtil * kp * cos(theta) + Wtil * kp * sin(theta)
    #     W = -Vtil * kp * sin(theta) + Wtil * kp * cos(theta) * (1/L)

    #     if V > BUGER_MAX_LIN_LEVEL:
    #         V = BUGER_MAX_LIN_LEVEL
    #     elif V < -BUGER_MAX_LIN_LEVEL:
    #         V = -BUGER_MAX_LIN_LEVEL
    #     else:
    #         V = V
        
    #     if W > BUGER_MAX_ANG_LEVEL:
    #         W = BUGER_MAX_ANG_LEVEL
    #     elif W < -BUGER_MAX_ANG_LEVEL:
    #         W = -BUGER_MAX_ANG_LEVEL
    #     else:
    #         W = W

    #     twist.linear.x = V
    #     twist.angular.z = W

    # 各エージェントに指令値を送る関数
    def main(self):

        # まず離陸させる，高さはclass呼び出し時に設定
        self.allcfs.takeoff(targetHeight=self.hight_start, duration=2.0)
        rospy.sleep(5)  # 5秒静止

        # 実験開始
        print("Experiment Start!!")
        start_time = time.time() # 開始時刻の取得，実験時間を図るため
        dt = 0.001
        while True:
            # 各crazyflieに指令を送る, !!!!!!!!!! 子フレーム(cf?)と命令対象のcrazyflieが一致している事が絶対条件 !!!!!!!!
            # child_frameはまだ何かわからないものであり、self.child_framesからcf1, cf2...と出力される
            for cf, child_frame in zip(self.allcfs.crazyflies, self.child_frames):
                
                ###
                # 原点と一つのcrazyflieとの座標変換を取得
                try:
                    # self.world_frameはワールド座標のこと
                    c = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))
                    t = self.tfBuffer.lookup_transform(self.world_frame, self.turtle_frame, rospy.Time(0))
                    # 以下のtfの使い方は使用できなさそう
                    # t = tf.transformations.euler_from_quaternion((self.data.pose.orientation.x, self.data.pose.orientation.y, self.data.pose.orientation.z, self.data.pose.orientation.w))

                # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr('LookupTransform Error !')
                    rospy.sleep(0.5)
                    continue

            # turtlebot3における変換
                # クオータニオン(四元数)の取得
                self.Quaternion_t = (t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
                # オイラー角の取得
                self.RPY_t = tf_conversions.transformations.euler_from_quaternion(self.Quaternion_t)  

                # self.Xp = t.transform.translation.x
                # self.Yp = t.transform.translation.y
                self.Xp = t.pose.position.x
                self.Yp = t.pose.position.z
                # turtlebot3などの障害物の頭上にcrazyflieを真上に持ってくると降下してしまう
                # Y方向に10cmほどずらす作業を行っている
                self.pos_des = list((float(self.Xp), float(self.Yp)+0.2, float(1.2))) 
                # self.pos_des = list((float(0), float(0) + 0.1, float(1.2)))        
                
            # crazyflieにおける変換
                # クオータニオン(四元数)の取得
                self.Quaternion = (c.transform.rotation.x,c.transform.rotation.y,c.transform.rotation.z,c.transform.rotation.w)
                # オイラー角の取得
                self.RPY_c = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)
                # 同次座標の取得
                self.homogerous_matrixs = tf_conversions.transformations.quaternion_matrix(self.Quaternion) + np.array([[0, 0, 0, c.transform.translation.x],
                                                                                                                        [0, 0, 0, c.transform.translation.y],
                                                                                                                        [0, 0, 0, c.transform.translation.z],
                                                                                                                        [0, 0, 0, 0]])
                                                            
                # pub.publish(twist)
                # rate.sleep()
                
                # コントローラー(vel_controller.py)には現在の位置，姿勢(yaw角)，速度, と目標の位置, 姿勢(yaw角)，速度を渡して指令値を更新する.
                self.cmd_controller_output(pos_now=self.homogerous_matrixs[:3, 3], pos_des=np.array(self.pos_des), 
                                           yaw_now=self.RPY_c[2], yaw_des=0.0, dt=dt)
                
                # エージェントに速度指令を送る, Zはコンスタントに小さな値を入れておかないとまずいかも
                # print(np.array([self.X_dot, self.Y_dot,self.Z_dot]), self.yaw_dot)
                print("X_dot{}".format(self.X_dot))
                print("Y_dot{}".format(self.Y_dot))
                print("Z_dot{}".format(self.Z_dot))
                cf.cmdVelocityWorld(np.array([self.X_dot, self.Y_dot, (0.01 + self.Z_dot)]), yawRate=self.yaw_dot)

                # 速度を記録する



                # 三次元位置を記録
                self.position_X.append(c.transform.translation.x)
                self.position_Y.append(c.transform.translation.y)
                self.position_Z.append(c.transform.translation.z)

                # PIDの微分項ようにサンプリング時間を設定，サンプリング時間は0.001秒*エージェントの数　+ 実行時間幅(ここでは切り捨ててる)
                rospy.sleep(dt)
            
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

        # 記録した三つ位置の描画
        fig, axes = plt.subplots(2, 1, figsize=(9, 14))

        axes[0].plot(self.position_X, self.position_Y, label='Px:{}, Ix:{}, Dx:{} \n Px:{}, Ix:{}, Dx:{}'.format(self.Xp, self.Xi, self.Xd, self.Yp, self.Yi, self.Yd))
        # axes[0].set_xtitle("X[m]")
        # axes[0].set_ytitle('Y[m]')
        
        axes[1].plot([n*0.001 for n in range(len(self.position_Z))], self.position_Z, label='Pz:{}, Iz:{}, Dz:{}'.format(self.Zp, self.Zi, self.Zd))
        # axes[1].set_ytitle("Z[m]")
        # axes[1].set_xtitle("T[s]")
        plt.legend()
        plt.show()
        

if __name__ == '__main__':
    # 初期位置の入力
    experiment_time = int(input("実験時間:"))
    hight_start = float(input("init_Z:"))
    # position_destination = list(float(0.0), float(0.0), float(0.5))
    position_destination = list((float(input("dest_X:")), float(input("dest_Y:")), float(input("dest_Z:"))))

    # rospy.init_node("position_control", anonymous=True)
    # rate = rospy.Rate(10)
    # pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    # rospy.Subscriber("/mocap_node/turtlebot3/pose", PoseStamped, const_value_control.callback)
    
    # const_value_controlを初期化し，main関数を実行
    # const_value_control(experiment_time, hight_start, position_destination).main()
    const_value_control(experiment_time, hight_start, position_destination).main()
    

# 実行方法

""" turtlebot3をReset Pivotしないと上手く動作しない """
""" turtlebot3の使用時間はおよそ1時間半ほど """

# $ cd ~/crazyswarm/ros_ws/src/crazyswarm/scripts/
# $ python chooser.py
# 使用するcrazyflieの番号を選択する

# roslaunch mocap_optitrack mocap.launch (情報を得る)
# roslaunch crazyswarm hover_swarm.launch (Rvizが表示される)
# turtlebot3がRvizの方で表示される(消さないようにすること)
# cloverや他のものが表示されてしまう場合は、いらないものをすべてコメントアウトし、もう一度mocap.launchを行う

# Data Streaming PaneからOptitrackで配信するRigid Bodiesをオンにする 
# Asset Markersをオフにする(crazyflieが読み取られない)
# Z Upにしないとcrazyflieが利用できない(turtlebot3側のYとZが逆転するので、注意すること)
# 通常のturtlebot3のみであれば、Y UpでOK
# 右にx、奥にy
# crazyflieを置く際に矢印がx軸が正の方向を向くようにする

# $ cd ~/crazyswarm/ros_ws/src/crazyswarm/turtlebot3/ (実行したい箇所まで移動する)
# $ export PYTHONPATH=$PYTHONPATH:~/crazyswarm/ros_ws/crazyswarm/scripts (PATHを通す)
# $ rosrun turtlebot3_ros ファイル名 (実行する)
# $ python tracking_control.py (こっちで実行するべきかもしれない)

# $ rosservice call /land tabキー
# エラーが起こった際に所望の高さなどで停止できる？

# 衝突などで実行が止まってしまった場合は、実行しているものを一旦区切る
# その後、$ python chooser.pyでrebootを行う必要がある

# 充電を確認したい際には、実行しているものを一旦区切る
# その後、$ python chooser.pyでbatteryを確認する(黄色/赤色だとNG)

# 台数番号によって、初期位置が定められている
# /crazyswarm/ros_ws/src/crazyswarm/flocking_ctrl/launch/allCrazyflies.yaml


"""
～方針～
元々完成しているcrazyflieの軌道追従制御をベースにして、turtlebot3の定置制御を組み込む
crazyflieはcrazyswarmの通信方法で、turtlebot3はTopic通信を行う

turtlebot3の座標はOptitrackのpose？などから取れるので、その座標を更新しつつ、その座標に向かってcrazyflieが動作させるようにする
crazyflieの初期位置はturtlebot3の初期位置と同じにする

turtlebot3の制御を終えたあとに、crazyflieを前方などに移動させて実験を終えたい
while not rospy.is_shutdown():を先頭のwhileに設定し、ifで所望の実験時間を超えた場合にbreakするようにする

(1) まずcrazyflieがturtlebot3の真上を追従するプログラムを作る
    turtlebot3はteleopなどで動かして
    crazyflieを着陸させるときに、turtlebot3の現在位置から？mだけずらした箇所に移動するようにする
    上手く処理されなかった場合に安全に着陸させるようにするのも大事である
(2) 完成したらturtlebot3を定置制御できるように組み込む

～問題点～
どのようにturtlebot3をcrazyflie側に認識させて、それを追従させるか
turtlebot3のノードの座標を追従するようにする？(zのパラメータだけを考慮しないで)
座標をappend？などに格納して、その値に追従するようにする？

最後のwhileを抜けるためのbreakが謎である

crazyflieに何秒ごとに入力を入れているのか、周期なども考えなくてはならない
ちなみにかとけいが作ったプログラムは10msに1入力

〜問題点(11/11)〜
大きい画面のパソコンのROS_MASTERのIPアドレスをいじらないといけない
プログラムはまだ怪しそう

"""