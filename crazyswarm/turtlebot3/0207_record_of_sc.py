#!/user/bin/env python
# coding: UTF-8

import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from scipy.spatial import Voronoi
from shapely.geometry import Polygon
import pandas as pd
import datetime

import matplotlib.animation as animation

# from ros_ws.src.crazyswarm.scripts.pycrazyswarm.crazyswarm import Crazyswarm
from pycrazyswarm import *

import rospy
import tf2_ros
import tf_conversions
import tf
# from crazyflie_driver import FullState

# 関連モジュールのインポート
from sc_coverage_controller import Coverage_controller
from sc_frames_setup import Frames_setup
# from coverage_voronoi import Coverage_voronoi

############################## 注意事項 ####################################

# turtlebot3を最初にxが正の向きに置かないとエラーが生じるので要注意する

############################## 被覆制御 ###############################################

class coverage_control(Frames_setup, Coverage_controller):

############################## 被覆制御のための知識 ####################################

# ボロノイ分割: 隣り合う母点間を結ぶ直線に垂直二等分線を引き、各母点の最近隣領域を分割する手法
# ボロノイ図(領域): ボロノイ分割によって作成される図
# ボロノイ辺: それぞれの母点間を結ぶ垂直二等分線のこと
# ボロノイ点(頂点): ボロノイ辺が交わる点のこと
# 母点: ある距離空間上の任意の位置に配置された複数個の点
# 有界: 集合が無限に遠くの要素を含まないことを言い表す言葉
# 閉空間: a <= x <= bのこと

#################### 有界なボロノイ図を計算・描写するための関数 #########################

    def get_voronoi(self, bnd, pnts):
        """ bnd  : ボロノイ分割する領域・範囲のための位置座標 """
        """ pnts : 母点座標 """

        # ボロノイ図の計算
        # 母点を与えるとボロノイ図を作成してくれるモジュール
        self.vor = Voronoi(pnts)

        # 分割する領域をPolygonにする，有界領域
        # 一番外枠の範囲を指定している
        # plt.Polygon((x1, y1), (x2, y2), (x3, y3), (x4, y4))とplt.show()で指定して範囲の図を描写できる
        self.bnd_poly = Polygon(bnd)
        # 各ボロノイ領域をしまうリスト
        self.vor_polys = []
        # 各ボロノイ領域のpolygonの頂点座標を格納
        self.poly_vertice = []
        # 各ボロノイ領域の平面座標系の頂点座標を格納
        self.pos_vertice = []

        for i in range(len(pnts) - 3):
        # range: range(6-3)であれば、0, 1, 2の最初の要素しか見ないことになる
        # for i in range(len(gn_pnts)):
            # 閉空間を考慮しないボロノイ領域
            # ダミー母点をなくすために-3という処理を行っている
            
            """ vertices(頂点): ボロノイ頂点座標を返す """
            """ regions: すべてのボロノイ領域を格納している """
            """ point_region: ボロノイ領域の点が格納されている """
            """ region(使用されていない？): ボロノイ領域 """

            # 結局ボロノイ領域の各頂点座標を取得しているだけ
            # ここではまだ重複を含めたボロノイ頂点を取得していると思われる

            self.vor_poly = [self.vor.vertices[v] for v in self.vor.regions[self.vor.point_region[i]]]

            # 分割する領域をボロノイ領域の共通部分を計算
            # ボロノイ領域の共通部分: まだ計算途中なので、ひとつひとつの母点の領域が重複してしまっている状態のこと
            # 重複なしのボロノイ頂点を取得

            """ intersection: 積集合(AかつB) """

            i_cell = self.bnd_poly.intersection(Polygon(self.vor_poly))
            self.pos_vertice.append(i_cell.exterior.coords[:-1])
            self.poly_vertice.append(Polygon(i_cell.exterior.coords[:-1]))

            # 閉空間を考慮したボロノイ領域の頂点座標を格納
            self.vor_polys.append(list(i_cell.exterior.coords[:-1]))

        # 各母点の位置，ボロノイ図の領域，ボロノイ図の各頂点，ボロノイ図の各頂点(座標平面に適応済み)
        """ gn_pnts = [[x1, y1], …, [xn, yn]]: 各母点の位置座標(n=crazyflieの台数) """
        """ vor_polys: ボロノイ図の領域 """
        """ poly_vertice: ボロノイ頂点の位置座標 """

        # gn_pnts = range(len(gn_pnts) - 3)
        return self.vor_polys, self.poly_vertice, self.pos_vertice

#################### ボロノイ図の描写のための関数 ########################################

    def plot_figure(self, k, bnd, p1, p2, fig, ax):

        ax.clear()
        xmin = np.min(bnd[:, 0])
        xmax = np.max(bnd[:, 0])
        ymin = np.min(bnd[:, 1])
        ymax = np.max(bnd[:, 1])
        # ax.set_xlim(xmin-0.1, xmax+0.1)
        # ax.set_ylim(ymin-0.1, ymax+0.1)
        ax.set_xlim(-0.5, 3.5)
        ax.set_ylim(-0.5, 3.5)
        ax.set_aspect('equal')
        # 母点の描画
        ax.scatter(p1[k][:-3][:, 0], p1[k][:-3][:, 1])
        # ボロノイ領域の描画
        # print(p2[0][k])
        poly_vor = PolyCollection(verts=p2[k], edgecolor="black", facecolors="None", linewidth=1.0)
        ax.add_collection(poly_vor)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        plt.title("coverage" + str(k+1))

        fig.canvas.draw()
        # fig.savefig("img.png")
        # ani = animation.FuncAnimation(fig, self.plot_figure, interval=1, frames=1000)
        # ani.save("output.mp4", writer="ffmpeg", fps=30, bitrate=1000)
        # ani.save("output.gif", writer="imagemagick")

#################### 重心を求めるための関数 ############################################

    def get_centroid(self, poly_gem, pos_gem):

        # 重心を求める

        centroid = []
        for i in range(len(list(poly_gem))):
            centroid.append(poly_gem[i].centroid)
        
        return centroid

############################## 初期設定のための関数 ####################################

    def __init__(self, experiment_time, hight_start):

        # frames_setup, vel_controllerの初期化
        super(Frames_setup, self).__init__()
        super(Coverage_controller, self).__init__()
        # super(Coverage_voronoi, self).__init__()

    # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = hight_start
        # 座標位置
        self.position_X = []
        self.position_Y = []
        self.position_Z = []
        self.T = []

    # ドローンの座標位置の記録用のリストの用意
        self.drone1_x = []
        self.drone1_y = []
        self.drone1_z = []
        self.drone2_x = []
        self.drone2_y = []
        self.drone2_z = []
        self.drone3_x = []
        self.drone3_y = []
        self.drone3_z = []
        self.drone4_x = []
        self.drone4_y = []
        self.drone4_z = []

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
    
    # tfフレームの初期化，フレーム間(原点と個々のcrazyflie)の位置関係が取得可能になる
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        time.sleep(1)
    
    # crazyflieの個体数も記録
        self.num_cf = len(self.child_frames)

        ########## pntsの初期化 ##########

        # crazyflieの位置座標を格納するための配列を用意する(後に最初の配列の要素を取り除く)
        self.pnts_init = np.array([0,0])
        # ダミーの母点を用意する
        # 3桁以上でないとエラーが出てしまうので要注意
        self.fake_pnts = np.array([[100, 100], [100, -100], [-100, 0]])

    # 各crazyflieに指令を送る!子フレーム(cf?)と命令対象のcrazyflieが一致している事が絶対条件 !
        
        for cf, child_frame in zip(self.allcfs.crazyflies, self.child_frames):
                
                # 原点と一つのcrazyflieとの座標変換を取得
            try:
                t = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))

                # 各crazyflyのx，y座標を配列に縦に結合していく
                self.pnts_init = np.vstack((self.pnts_init, np.array([t.transform.translation.x,t.transform.translation.y])))
                print(self.pnts_init)

                # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Error !')
                rospy.sleep(0.5)
                continue   

        # 最初に入れた一行目の要素を取り除いている
        self.pnts_init = self.pnts_init[1:, :]
        # 母点の初期位置を確認する
        print(self.pnts_init)

############################## メインで実行される関数 ####################################

    # 各エージェントに指令値を送る関数
    def main(self):
        
        print("experiment start!")

        # まず離陸させる，高さはclass呼び出し時に設定
        # targetHeightは目標の高さ, durationは離陸にかける時間
        self.allcfs.takeoff(targetHeight=self.hight_start, duration=3.0)
        rospy.sleep(5)  # 5秒静止

        # 実験開始
        start_time = time.time() # 開始時刻の取得，実験時間を図るため
        dt=0.001
        k = 0

        # 母点の位置
        self.pnts = self.pnts_init

        ########## 動的なグラフを作成するための初期設定 ##########

        # 下記のコマンドで画像の比率を決める
        fig = plt.figure(figsize=(7, 6))
        # 下記のコマンドで描画領域が追加される
        ax = fig.add_subplot(111)
        # 下記のコマンドで動くプロットが可能となる
        # plt.show()した時点であとから要素を書き足すことができなくなる
        plt.ion()
        # plt.show()と同義
        fig.show()
        p1 = []
        p2 = []
        # fig.canvas.draw()

        while True:
            
            # 一次的に各crazyflyの位置を記憶する配列
            self.tmp_pnts = np.array([0, 0])
            # ここで母点を有界にするためにダミー母点三個追加する？？？？
            self.pnts = np.concatenate([self.pnts, self.fake_pnts])

            turtle = self.tfBuffer.lookup_transform(self.world_frame, self.turtle_frame, rospy.Time(0))

            ########## turtlebot3のtf変換 ##########

            # クオータニオン(四元数)の取得
            self.Quaternion_t = (turtle.transform.rotation.x,turtle.transform.rotation.z,-turtle.transform.rotation.y,turtle.transform.rotation.w)
            # オイラー角の取得
            self.RPY_t = tf_conversions.transformations.euler_from_quaternion(self.Quaternion_t)  
            # turtlebot3の位置座標を変数に入れる(ZUpなので, y→Zに変換している)
            self.turtle_state_X = turtle.transform.translation.x
            self.turtle_state_Y = turtle.transform.translation.z

            ########## ボロノイ図を計算 ##########
                
            # turtlebot3の位置座標を中心とした領域の設定のための準備
            pos_plus_x = self.turtle_state_X + 1.2
            pos_plus_y = self.turtle_state_Y + 1.2
            pos_minus_x = self.turtle_state_X + (-1.2)
            pos_minus_y = self.turtle_state_Y + (-1.2)
            # ボロノイ分割する領域を設定する
            # bnd = np.array([[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5]])
            bnd = np.array([[pos_minus_x, pos_minus_y],
                            [pos_plus_x , pos_minus_y], 
                            [pos_plus_x , pos_plus_y],
                            [pos_minus_x, pos_plus_y]])

            # 初期位置はおよそ[2.2, 1.7]
            # 最初の被覆制御のためのボロノイ分割する領域を指定してあげる
            # bnd = np.array([[3.2, 2.7], [3.2 , 0.7], [1.2 , 2.7], [1.2, 0.7]])

            # ボロノイ図の計算・描画
            vor_polys, poly_vertice, pos_vertice = self.get_voronoi(bnd, self.pnts)

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
                    c = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))
                    self.tmp_pnts = np.vstack((self.tmp_pnts, np.array([c.transform.translation.x, c.transform.translation.y])))
                    # turtlebot3の位置座標を取得する
                    turtle = self.tfBuffer.lookup_transform(self.world_frame, self.turtle_frame, rospy.Time(0))

                # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr('LookupTransform Error !')
                    rospy.sleep(0.5)
                    continue   

                # ドローンの座標位置を記録する
                if i == 0:
                    self.drone1_x.append(c.transform.translation.x)
                    self.drone1_y.append(c.transform.translation.y)
                    self.drone1_z.append(c.transform.translation.z)
                elif i == 1:
                    self.drone2_x.append(c.transform.translation.x)
                    self.drone2_y.append(c.transform.translation.y)
                    self.drone2_z.append(c.transform.translation.z)
                elif i == 2:
                    self.drone3_x.append(c.transform.translation.x)
                    self.drone3_y.append(c.transform.translation.y)
                    self.drone3_z.append(c.transform.translation.z)
                elif i == 3:
                    self.drone4_x.append(c.transform.translation.x)
                    self.drone4_y.append(c.transform.translation.y)
                    self.drone4_z.append(c.transform.translation.z)

            ########## crazyflieのtf変換 ##########

            # クオータニオン(四元数)の取得
                self.Quaternion = (c.transform.rotation.x, c.transform.rotation.y, c.transform.rotation.z, c.transform.rotation.w)
            # オイラー角の取得
                self.RPY = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)
            # 同次座標の取得
                self.homogerous_matrixs = tf_conversions.transformations.quaternion_matrix(self.Quaternion) + np.array([[0, 0, 0, c.transform.translation.x],
                                                                                                                        [0, 0, 0, c.transform.translation.y],
                                                                                                                        [0, 0, 0, c.transform.translation.z],
                                                                                                                        [0, 0, 0, 0]])

            ########## turtlebot3のtf変換 ##########

            # クオータニオン(四元数)の取得
                self.Quaternion_t = (turtle.transform.rotation.x,turtle.transform.rotation.z,-turtle.transform.rotation.y,turtle.transform.rotation.w)
            # オイラー角の取得
                self.RPY_t = tf_conversions.transformations.euler_from_quaternion(self.Quaternion_t)  
            # turtlebot3の位置座標を変数に入れる(ZUpなので, y→Zに変換している)
                self.turtle_state_X = turtle.transform.translation.x
                self.turtle_state_Y = turtle.transform.translation.z

            ########## ボロノイ図を計算 ##########
                
            # turtlebot3の位置座標を中心とした領域の設定のための準備
                pos_plus_x = self.turtle_state_X + 1.2
                pos_plus_y = self.turtle_state_Y + 1.2
                pos_minus_x = self.turtle_state_X + (-1.2)
                pos_minus_y = self.turtle_state_Y + (-1.2)
            # ボロノイ分割する領域を設定する
                # bnd = np.array([[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5]])
                bnd = np.array([[pos_minus_x, pos_minus_y],
                                [pos_plus_x , pos_minus_y], 
                                [pos_plus_x , pos_plus_y],
                                [pos_minus_x, pos_plus_y]])

            ########## 重心位置の計算 ##########

            # 母点の個数
                # n = self.num_cf
            # 母点(crazyflieの台数を確認する)
                # print(n)

            # ボロノイ図の計算・描画        
                # vor_polys, poly_vertice, pos_vertice = self.get_voronoi(bnd, self.pnts)

            # グラフの描画
                # self.plot_figure(k, bnd, p1, p2, fig, ax)

            # アニメーション作成に必要なリストを作成
                # p1.append(self.pnts)
                # p2.append(vor_polys)

            # 重心を求める
                # centroid = self.get_centroid(poly_vertice, pos_vertice)

            ########## シミュレーション用のコマンド ##########

            # 重心との偏差を計算する
                # x_err, y_err = self.get_err(self.pnts, centroid)

            # 偏差から一度に動く量を計算する
                # x_move, y_move = self.get_move(x_err,y_err)

            ########## crazyflieに送る目標値の設定 ##########

            # 目標値生成
                hight_desired = hight_start
                # x,yは重心の位置, zは所望の高さを入れる
                pos_desired = [np.array(centroid[i])[0] ,np.array(centroid[i])[1] , hight_desired]
            
            # コントローラーに現在の位置, 姿勢, 目標の位置, 姿勢, 速度指令値の更新
                self.cmd_controller_output(pos_now=self.homogerous_matrixs[:3, 3], yaw_now=self.RPY[2], pos_des=pos_desired, yaw_des=0.0, dt=dt)
            
            # crazyflieに速度指令を送る
                cf.cmdVelocityWorld(np.array([self.X_dot, self.Y_dot, self.Z_dot]), yawRate=self.yaw_dot)
                # print(self.X_dot, self.Y_dot, self.Z_dot, self.yaw_dot)

            # 三次元位置を記録
                self.position_X.append(self.homogerous_matrixs[0, 3])
                self.position_Y.append(self.homogerous_matrixs[1, 3])
                self.position_Z.append(self.homogerous_matrixs[2, 3])
                self.T.append(time.time() - start_time)               

            ########## 位置情報のいらない情報を抜き取る ##########

            # 一次的に記憶した各crazyflyの位置情報から最初のいらない情報だけ抜き出す
            # while文の中に入れてしまうと上の要素をひとつずつ消していってしまう
            self.tmp_pnts = self.tmp_pnts[1:, :]
            self.pnts = self.tmp_pnts

            k += 1
            print(k)

            ########## 実験を終了させるためのルールの取り決め ##########

            # 実験時間が実験終了時間を過ぎたら機体を着陸させる
            if time.time() - start_time > self.exp_time:

                # 速度指令を送ったあとに着陸指令を送るときは, この関数を個々のcrazyflieに対して呼び出す必要がある
                for cf in self.allcfs.crazyflies:
                    cf.notifySetpointsStop(100)

                # 着陸, targetHeightはプロペラ停止高度, durationは着陸にかける時間
                self.allcfs.land(targetHeight=0.02, duration=4.0)
                rospy.sleep(5) # 5秒停止
                print(time.time() - start_time)
                print("experiment finish!!")

                break

        ########## データの記録 ##########

        ani = animation.FuncAnimation(fig, self.plot_figure, fargs=(bnd, p1, p2, fig, ax), 
                                      frames=k, interval=100)
        ani.save("0214_sc_data2.mp4", writer="ffmpeg")
        plt.show()

        dict = {'drone1_x': self.drone1_x, 'drone1_y': self.drone1_y, 'drone1_z': self.drone1_z, 
                'drone2_x': self.drone2_x, 'drone2_y': self.drone2_y, 'drone2_z': self.drone2_z, 
                'drone3_x': self.drone3_x, 'drone3_y': self.drone3_y, 'drone3_z': self.drone3_z,
                'drone4_x': self.drone4_x, 'drone4_y': self.drone4_y, 'drone4_z': self.drone4_z,}
        df = pd.DataFrame(dict)
        df.to_csv('0214_sc_data2.csv')


        # data = {"T": self.T, "X": self.position_X, "Y":self.position_Y,"Z": self.position_Z}

        # df = pd.DataFrame(data)
        # df.to_csv("coverage_ctrl_after{}".format(datetime.date.today()))

        # fig = plt.figure()
        # ax1 = fig.add_subplot(1, 1, 1)
        # ax1.plot(self.position_X, self.position_Y)
        # ax1.set_xlabel('X[m]')
        # ax1.set_ylabel('Y[m]')
        # ax1.set_xlim(-1.5, 1.5)
        # ax1.set_ylim(-1.5, 1.5)
        # plt.grid()
        # plt.show()

############################## 最初に実行される関数 ####################################

if __name__ == '__main__':
    
    # 初期情報の入力
    experiment_time = int(input("実験時間:"))
    hight_start = float(input("init_Z:"))
    # position_destination = list((float(input("dest_X:")), float(input("dest_Y:")), float(input("dest_Z:"))))
    
    # const_value_controlを初期化し，main関数を実行
    coverage_control(experiment_time, hight_start).main()