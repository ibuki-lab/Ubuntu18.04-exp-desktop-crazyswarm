#!/user/bin/env python
# coding: UTF-8

import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from scipy.spatial import Voronoi
from shapely.geometry import Polygon

# from ros_ws.src.crazyswarm.scripts.pycrazyswarm.crazyswarm import Crazyswarm
from pycrazyswarm import *

import rospy
import tf2_ros
import tf_conversions
import tf
# from crazyflie_driver import FullState

# 関連モジュールのインポート
from coverage_controller import Coverage_controller
from frames_setup import Frames_setup
from coverage_voronoi import Coverage_voronoi

############## 被覆制御 ##############################################################################


class coverage_control(Frames_setup, Coverage_controller, Coverage_voronoi):

    ##### このクラスの初期設定を行う関数 ##############################################################

    def __init__(self, experiment_time, hight_start):

        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__()
        super(Coverage_voronoi, self).__init__()

    # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = hight_start
        self.position_X = []
        self.position_Y = []
        self.position_Z = []
        self.T = []

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

    # tfフレームの初期化，フレーム間(原点と個々のcrazyflie)の位置関係が取得可能になる
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        time.sleep(1)

    # crazyflieの個体数も記録
        self.num_cf = len(self.child_frames)

    # 飛行していないcrazyflie
        self.land_cfs = []

    # 飛行中のcrazyflie
        self.fly_cfs = self.child_frames


############pntsの初期化#########################################################################

        self.pnts_init = np.array([0, 0])
        self.fake_pnts = np.array([[100, 100], [100, -100], [-100, 0]])

    # 各crazyflieに指令を送る!子フレーム(cf?)と命令対象のcrazyflieが一致している事が絶対条件 !

        for cf, child_frame in zip(self.allcfs.crazyflies, self.child_frames):

            # 原点と一つのcrazyflieとの座標変換を取得
            try:
                t = self.tfBuffer.lookup_transform(
                    self.world_frame, child_frame, rospy.Time(0))
                print(self.pnts_init)
                self.child_frames
                # 各crazyflyのx，y座標を配列に縦に結合していく
                self.pnts_init = np.vstack((self.pnts_init, np.array(
                    [t.transform.translation.x, t.transform.translation.y])))

                # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Error !')
                rospy.sleep(0.5)
                continue

        self.pnts_init = self.pnts_init[1:, :]
        print(self.pnts_init)

########ボロノイ分割##############################################################################
    # 各エージェントに指令値を送る関数
    def main(self):

        # まず離陸させる，高さはclass呼び出し時に設定
        # 指定したcrazyflieのみ離陸
        for i, cf, child_frame in zip(range(self.num_cf), self.allcfs.crazyflies, self.child_frames):

            if child_frame != 'cf2':
                cf.takeoff(targetHeight=self.hight_start, duration=2.0)

        rospy.sleep(5)  # 5秒静止

        self.land_cfs.append("cf2")
        self.fly_cfs.remove("cf2")

        # 実験開始
        start_time = time.time()  # 開始時刻の取得，実験時間を図るため
        dt = 0.001
        k = 0
        num = 0
        # 離陸していないcrazyflieを考慮しない
        self.pnts_init = np.delete(
            self.pnts_init, 1, 0)

        # 母点の位置
        self.pnts = self.pnts_init

        # 動的なグラフを作成するための初期設定
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(111)
        plt.ion()
        fig.show()
        fig.canvas.draw()

        # アニメーション作成用リスト
        p1 = []
        p2 = []

        while True:

            # 一次的に各crazyflyの位置を記憶する配列
            self.tmp_pnts = np.array([0, 0])
            # ここで母店を有界にするためにダミー母店を三個追加する？？？？
            self.pnts = np.concatenate([self.pnts, self.fake_pnts])

            # ボロノイ図を計算

            # ボロノイ分割する領域
            bnd = np.array(
                [[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5]])

            # 母点の個数
            n = self.num_cf

            # ボロノイ図の計算・描画
            vor_polys, poly_vertice, pos_vertice = self.get_voronoi(
                bnd, self.pnts)

            # アニメーション作成に必要なリストを作成
            p1.append(self.pnts)
            p2.append(vor_polys)

            # グラフの描画
            self.plot_figure(k, bnd, p1, p2, fig, ax)

            # 重心を求める
            centroid = self.get_centroid(poly_vertice, pos_vertice)

            # 各crazyflieに指令を送る!子フレーム(cf?)と命令対象のcrazyflieが一致している事が絶対条件 !
            for i, cf, child_frame in zip(range(self.num_cf), self.allcfs.crazyflies, self.child_frames):

                # 原点と一つのcrazyflieとの座標変換を取得
                try:
                    t = self.tfBuffer.lookup_transform(
                        self.world_frame, child_frame, rospy.Time(0))
                    self.tmp_pnts = np.vstack((self.tmp_pnts, np.array(
                        [t.transform.translation.x, t.transform.translation.y])))

                # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr('LookupTransform Error !')
                    rospy.sleep(0.5)
                    continue

######## ボロノイ分割をして重心（目標値を求める） ##########################################################################################

            # クオータニオン(四元数)の取得
                self.Quaternion = (t.transform.rotation.x, t.transform.rotation.y,
                                   t.transform.rotation.z, t.transform.rotation.w)
            # オイラー角の取得
                self.RPY = tf_conversions.transformations.euler_from_quaternion(
                    self.Quaternion)
            # 同次座標の取得
                self.homogerous_matrixs = tf_conversions.transformations.quaternion_matrix(self.Quaternion) + np.array([[0, 0, 0, t.transform.translation.x],
                                                                                                                        [0, 0, 0, t.transform.translation.y],
                                                                                                                        [0, 0, 0, t.transform.translation.z],
                                                                                                                        [0, 0, 0, 0]])

###############################################################################################################
            # 着陸中のcrazyflieには命令を送らない or 条件が揃えば離陸命令
                if num == 0:

                    if time.time() - start_time < 20:

                        if child_frame == "cf2":

                            print("cf1は着陸中")

                            self.tmp_pnts = np.delete(
                                self.tmp_pnts, self.child_frames.index(child_frame)+1, 0)

                            # 三次元位置を記録
                            self.position_X.append(
                                self.homogerous_matrixs[0, 3])
                            self.position_Y.append(
                                self.homogerous_matrixs[1, 3])
                            self.position_Z.append(
                                self.homogerous_matrixs[2, 3])
                            self.T.append(time.time() - start_time)

                        if child_frame != "cf2":

                        # 目標値生成

                            hight_desired = hight_start
                            pos_desired = [np.array(centroid[self.fly_cfs.index(child_frame)])[0], np.array(
                                centroid[self.fly_cfs.index(child_frame)])[1], hight_desired]

                        # コントローラーに現在の位置，姿勢，目標の位置，姿勢 速度指令値の更新
                            self.cmd_controller_output(
                                pos_now=self.homogerous_matrixs[:3, 3], yaw_now=self.RPY[2], pos_des=pos_desired, yaw_des=0.0, dt=dt)

                        # crazyflieに速度指令を送る
                            cf.cmdVelocityWorld(
                                np.array([self.X_dot, self.Y_dot, self.Z_dot]), yawRate=self.yaw_dot)

                        # 三次元位置を記録
                            self.position_X.append(
                                self.homogerous_matrixs[0, 3])
                            self.position_Y.append(
                                self.homogerous_matrixs[1, 3])
                            self.position_Z.append(
                                self.homogerous_matrixs[2, 3])
                            self.T.append(time.time() - start_time)

                    if time.time() - start_time > 20:

                        if child_frame == "cf2":

                            print(str(child_frame)+"離陸開始")
                            cf.takeoff(
                                targetHeight=self.hight_start, duration=2.0)
                            num += 1
                            print(str(child_frame)+"離陸終了")

                        # 三次元位置を記録
                            self.position_X.append(
                                self.homogerous_matrixs[0, 3])
                            self.position_Y.append(
                                self.homogerous_matrixs[1, 3])
                            self.position_Z.append(
                                self.homogerous_matrixs[2, 3])
                            self.T.append(time.time() - start_time)

                            self.fly_cfs.append(child_frame)
                            self.land_cfs.remove(child_frame)

                        if child_frame != "cf2":

                            # 目標値生成

                            hight_desired = hight_start
                            pos_desired = [np.array(centroid[i])[0], np.array(
                                centroid[i])[1], hight_desired]

                        # コントローラーに現在の位置，姿勢，目標の位置，姿勢 速度指令値の更新
                            self.cmd_controller_output(
                                pos_now=self.homogerous_matrixs[:3, 3], yaw_now=self.RPY[2], pos_des=pos_desired, yaw_des=0.0, dt=dt)

                        # crazyflieに速度指令を送る
                            cf.cmdVelocityWorld(
                                np.array([self.X_dot, self.Y_dot, self.Z_dot]), yawRate=self.yaw_dot)

                        # 三次元位置を記録
                            self.position_X.append(
                                self.homogerous_matrixs[0, 3])
                            self.position_Y.append(
                                self.homogerous_matrixs[1, 3])
                            self.position_Z.append(
                                self.homogerous_matrixs[2, 3])
                            self.T.append(time.time() - start_time)

                if num == 1:

                    # 目標値生成

                    hight_desired = hight_start
                    pos_desired = [np.array(centroid[i])[0], np.array(
                        centroid[i])[1], hight_desired]

                # コントローラーに現在の位置，姿勢，目標の位置，姿勢 速度指令値の更新
                    self.cmd_controller_output(
                        pos_now=self.homogerous_matrixs[:3, 3], yaw_now=self.RPY[2], pos_des=pos_desired, yaw_des=0.0, dt=dt)

                # crazyflieに速度指令を送る
                    cf.cmdVelocityWorld(
                        np.array([self.X_dot, self.Y_dot, self.Z_dot]), yawRate=self.yaw_dot)

                # 三次元位置を記録
                    self.position_X.append(
                        self.homogerous_matrixs[0, 3])
                    self.position_Y.append(
                        self.homogerous_matrixs[1, 3])
                    self.position_Z.append(
                        self.homogerous_matrixs[2, 3])
                    self.T.append(time.time() - start_time)


##################################################################################################

            # 一次的に記憶した各crazyflyの位置情報から最初のいらない情報だけ抜き出す
            self.tmp_pnts = self.tmp_pnts[1:, :]
            self.pnts = self.tmp_pnts

            k += 1
            print(k, time.time() - start_time)

            # 実験時間が実験終了時間を過ぎたら機体を着陸させる
            if time.time() - start_time > self.exp_time:

                # 速度指令を送ってた後に着陸指令を送るときはこの関数を個々のcrazyflieに対し呼び出す必要がある
                for cf in self.allcfs.crazyflies:
                    cf.notifySetpointsStop(100)

                # 着陸, targethightはプロペラ停止高度，durationは着陸にかける時間
                self.allcfs.land(targetHeight=0.02, duration=4.0)
                rospy.sleep(5)  # 5秒停止
                print(time.time() - start_time)
                print("experiment finish!!")

                break

###############################################################################################


if __name__ == '__main__':

    # 初期情報の入力
    experiment_time = int(input("実験時間:"))
    hight_start = float(input("init_Z:"))
    # position_destination = list((float(input("dest_X:")), float(input("dest_Y:")), float(input("dest_Z:"))))

    # const_value_controlを初期化し，main関数を実行
    coverage_control(experiment_time, hight_start).main()


# 一台追加するプログラム
# 追加する順番は固定かもしれない