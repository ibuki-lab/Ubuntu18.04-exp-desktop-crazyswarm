#!/user/bin/env python
# coding: UTF-8

# turtlebot3の位置を中心とした範囲で、crazyflieが被覆制御を行うようなプログラムを作成したい

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
from aft_turtle_cc_frames_setup import Frames_setup

############################## 被覆制御 ###############################################

class coverage_control( Frames_setup, Coverage_controller):

############################## 被覆制御のための知識 ####################################

# ボロノイ分割: 隣り合う母点間を結ぶ直線に垂直二等分線を引き、各母点の最近隣領域を分割する手法
# ボロノイ図(領域): ボロノイ分割によって作成される図
# ボロノイ辺: それぞれの母点間を結ぶ垂直二等分線のこと
# ボロノイ点(頂点): ボロノイ辺が交わる点のこと
# 母点: ある距離空間上の任意の位置に配置された複数個の点
# 有界: 集合が無限に遠くの要素を含まないことを言い表す言葉
# 閉空間: a <= x <= bのこと


############################## 初期設定のための関数 ####################################
    
    def __init__(self, experiment_time, hight_start):

        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__()

    # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = hight_start
        # self.hight_start =0.5
        # self.pos_des = pos_des
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
        
    
    # tfフレームの初期化，フレーム間(原点と個々のcrazyflie)の位置関係が取得可能になる
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
    
    # crazyflieの個体数も記録
        self.num_cf = len(self.child_frames)

#################### 有界なボロノイ図を計算・描写するための関数 #########################

    def get_voronoi(self, bnd, pnts):

        """ bnd  : ボロノイ分割する領域・範囲のための位置座標 """
        """ pnts : 母点座標 """

        # すべての母点のボロノイ領域を有界にするために，ダミー母点を三個追加
        # concatenate: 行列を縦方向に結合する
        """ gn_pnts = [[x, y], [100, 100], [100, -100], [-100, 0]] """
        gn_pnts = np.concatenate([pnts, np.array([[100, 100], [100, -100], [-100, 0]])])
        # 下記の母点の座標だとなぜかエラーが発生する(3 tuplesが必要?)
        # gn_pnts = np.concatenate([pnts, np.array([[60, 60], [60, 10], [10, 60]])])
        # gn_pnts = np.concatenate([pnts])

        # ボロノイ図の計算
        # 母点を与えるとボロノイ図を作成してくれるモジュール
        vor = Voronoi(gn_pnts)

        # 分割する領域をPolygonにする，有界領域
        # 一番外枠の範囲を指定している
        # plt.Polygon((x1, y1), (x2, y2), (x3, y3), (x4, y4))とplt.show()で指定して範囲の図を描写できる
        bnd_poly = Polygon(bnd)
        # 各ボロノイ領域をしまうリスト
        vor_polys = []
        # 各ボロノイ領域のpolygonの頂点座標を格納
        poly_vertice = []
        # 各ボロノイ領域の平面座標系の頂点座標を格納
        pos_vertice = []

        for i in range(len(gn_pnts) - 3):
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

            vor_poly = [vor.vertices[v] for v in vor.regions[vor.point_region[i]]]

            # 分割する領域をボロノイ領域の共通部分を計算
            # ボロノイ領域の共通部分: まだ計算途中なので、ひとつひとつの母点の領域が重複してしまっている状態のこと
            # 重複なしのボロノイ頂点を取得

            """ intersection: 積集合(AかつB) """

            i_cell = bnd_poly.intersection(Polygon(vor_poly))
            pos_vertice.append(i_cell.exterior.coords[:-1])
            poly_vertice.append(Polygon(i_cell.exterior.coords[:-1]))

            # 閉空間を考慮したボロノイ領域の頂点座標を格納
            vor_polys.append(list(i_cell.exterior.coords[:-1]))

        # 各母点の位置，ボロノイ図の領域，ボロノイ図の各頂点，ボロノイ図の各頂点(座標平面に適応済み)
        """ gn_pnts = [[x1, y1], …, [xn, yn]]: 各母点の位置座標(n=crazyflieの台数) """
        """ vor_polys: ボロノイ図の領域 """
        """ poly_vertice: ボロノイ頂点の位置座標 """

        # gn_pnts = range(len(gn_pnts) - 3)
        return gn_pnts, vor_polys, poly_vertice

#################### 重心を求めるための関数 ############################################

    def get_centroid(self, poly_gem):

        # assert type(gem) == "shapely", "input should be a Shapely geometory"
        # assert gem.geom_type in ['Point', 'LineSrtring', 'Polygon'], "Input should be a Shapely geometry"
        # ポリゴンに変換
        # 重心を求める

        centroid = []
        for i in range(len(list(poly_gem))):
            centroid.append(poly_gem[i].centroid)
        
        return centroid

 #################### 求めた重心との偏差を求めるための関数 ##############################

    # 求めた重心との偏差を計算する
    def get_err(self, pnts, centroid):
        x_err = []
        y_err = []
        for i in range(len(pnts)):
            x_err.append(pnts[i][0] - np.array(centroid[i])[0] + 0.0001 * np.random.randn())
            y_err.append(pnts[i][1] - np.array(centroid[i])[1] + 0.0001 * np.random.randn())
        return x_err, y_err

#################### 求めた偏差から一度に動く量を計算するための関数 #####################

    # 求めた偏差から一度に動く量を計算する
    def get_move(self, x_err, y_err):
        gain = 0.1
        x_move = []
        y_move = []
        for i in range(len(x_err)):
            x_move.append(-gain*x_err[i] + 0.00* np.random.randn())
            y_move.append(-gain*y_err[i] + 0.00 * np.random.randn())
        return x_move, y_move 

#################### 動いたあとの位置を更新するための関数 ###############################

    # 動いた後の位置を更新
    def get_newpnts(self, pnts, x_move, y_move):
        for i in range(len(pnts)):
            pnts[i][0] += x_move[i]
            pnts[i][1] += y_move[i]
        return pnts

#################### ボロノイ図の描写のための関数 ########################################

    # ボロノイ図の描画
    def plot_figure(self, bnd, gn_pnts, vor_polys, fig, ax):
        
        ax.clear()
        xmin = np.min(bnd[:, 0])
        xmax = np.max(bnd[:, 0])
        ymin = np.min(bnd[:, 1])
        ymax = np.max(bnd[:, 1])
        ax.set_xlim(xmin-0.1, xmax+0.1)
        ax.set_ylim(ymin-0.1, ymax+0.1)
        ax.set_aspect('equal')
        # 母点の描画
        ax.scatter(gn_pnts[:-3][:, 0], gn_pnts[:-3][:, 1])
        # ボロノイ領域の描画
        """ verts: 各ポリゴンの頂点を二次配列で渡す[[2, 2], [2, 3]]みたいに """

        poly_vor = PolyCollection(verts=vor_polys, edgecolor="black", facecolors="None", linewidth=1.0)
        ax.add_collection(poly_vor)
        fig.canvas.draw()

#################### メインで実行される関数　###########################################


    # 各エージェントに指令値を送る関数
    def main(self):

        # まず離陸させる，高さはclass呼び出し時に設定
        # self.allcfs.takeoff(targetHeight=self.hight_start, duration=2.0)
        self.allcfs.takeoff(self.hight_start, 4)
        rospy.sleep(3)  # 3秒静止

        # 実験開始
        print("Experiment Start!!")
        start_time = time.time() # 開始時刻の取得，実験時間を図るため
        dt=0.001
        
        fig = plt.figure(figsize=(7, 6))
        # 下記のコマンドで描画領域が追加される
        ax = fig.add_subplot(111)
        # plt.ion: 動くプロットが可能になる
        plt.ion()
        #  plt.showした時点であとからプラスすることはできなくなってしまう
        fig.show()
        fig.canvas.draw()

        # ボロノイ分割する領域
        # ここでturtlebot3の座標を中心にした領域の範囲が指定できる
        # bnd = np.array([[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5]])

        # 母点の個数
        n = self.num_cf
        print(n)
        
        while True:
            # 各crazyflieに指令を送る!子フレーム(cf?)と命令対象のcrazyflieが一致している事が絶対条件 !
            for cf, child_frame in zip(self.allcfs.crazyflies, self.child_frames):
                
                # 原点と一つのcrazyflieとの座標変換を取得
                try:
                    c = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))
                    # crazyflieを一台ずつ情報を取得しているときに、turtlebot3も同時に行わない方が良い？
                    # 1/27では特にlookupエラーは出ない
                    turtle = self.tfBuffer.lookup_transform(self.world_frame, self.turtle_frame, rospy.Time(0))

                # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr('LookupTransform Error !')
                    rospy.sleep(0.5)
                    continue   
            
            # クオータニオン(四元数)の取得
                self.Quaternion = (c.transform.rotation.x,c.transform.rotation.y,c.transform.rotation.z,c.transform.rotation.w)
            # オイラー角の取得
                self.RPY = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)
            # 同次座標の取得
                self.homogerous_matrixs = tf_conversions.transformations.quaternion_matrix(self.Quaternion) + np.array([[0, 0, 0, c.transform.translation.x],
                                                                                                                        [0, 0, 0, c.transform.translation.y],
                                                                                                                        [0, 0, 0, c.transform.translation.z],
                                                                                                                        [0, 0, 0, 0]])

            # tryの中に入れてまうと上手くlookupができなくなる？
                # turtle = self.tfBuffer.lookup_transform(self.world_frame, self.turtle_frame, rospy.Time(0))
            # turtlebot3における変換
            # クオータニオン(四元数)の取得
                self.Quaternion_t = (turtle.transform.rotation.x,turtle.transform.rotation.z,-turtle.transform.rotation.y,turtle.transform.rotation.w)
            # オイラー角の取得
                self.RPY_t = tf_conversions.transformations.euler_from_quaternion(self.Quaternion_t)  
                self.turtle_state_X = turtle.transform.translation.x
                self.turtle_state_Y = turtle.transform.translation.z

            # ボロノイ分割する領域
            # ここでturtlebot3の座標を中心にした領域の範囲が指定できる
            # 配列の要素の個数が以前のものよりも多くなっている
                pos_plus_x = self.turtle_state_X + 1.5
                pos_plus_y = self.turtle_state_Y + 1.5
                pos_minus_x = self.turtle_state_X + (-1.5)
                pos_minus_y = self.turtle_state_Y + (-1.5)
                bnd = np.array([[pos_minus_x, pos_minus_y],
                                [pos_plus_x , pos_minus_y], 
                                [pos_plus_x , pos_plus_y],
                                [pos_minus_x, pos_plus_y]])

#################### ボロノイ分割をして重心(目標値)を求めるための関数 ##################

                # 座標[x, y]を格納している
                # 下記のままだと一次元配列なので、二次元配列に直す
                # pnts = self.homogerous_matrixs[:2, 3]

                pnts_x = self.homogerous_matrixs[0, 3]
                pnts_y = self.homogerous_matrixs[1, 3]
                pnts = np.array([[pnts_x, pnts_y]])
                print(pnts)

            # ボロノイ図を計算
                
                # ボロノイ図の計算・描画        
                # gn_pnts, vor_polys, poly_vertice, pos_vertice  = self.get_voronoi(bnd, pnts)
                gn_pnts, vor_polys, poly_vertice = self.get_voronoi(bnd, pnts)
                
                # グラフの描画
                self.plot_figure(bnd, gn_pnts, vor_polys, fig, ax)
                
                # 重心を求める
                centroid = self.get_centroid(poly_vertice)

                # 重心との偏差を計算する
                x_err, y_err = self.get_err(pnts, centroid)

                # 偏差から一度に動く量を計算する
                x_move, y_move = self.get_move(x_err,y_err)

                # 動いた後の位置を更新
                new_pnts = self.get_newpnts(pnts, x_move, y_move)

                # 目標値
                # listを用いているはずだから、下記のようなappendは使えないはず
                # pos_desired = new_pnts.append(self.hight_start)
                pos_desired = np.array([x_move, y_move, self.hight_start])
###############################################################################################################
            
            # コントローラーに現在の位置，姿勢，目標の位置，姿勢 速度指令値の更新
                self.cmd_controller_output(pos_now=self.homogerous_matrixs[:3, 3], yaw_now=self.RPY[2], pos_des=pos_desired, yaw_des=0.0, dt=dt)
            
            # crazyflieに速度指令を送る
                cf.cmdVelocityWorld(np.array([self.X_dot, self.Y_dot, self.Z_dot]), yawRate=self.yaw_dot)
                print(self.X_dot, self.Y_dot, self.Z_dot, self.yaw_dot)

##################################################################################################
                             
                
            # 実験時間が実験終了時間を過ぎたら機体を着陸させる
            if time.time() - start_time > self.exp_time:

                # 速度指令を送ってた後に着陸指令を送るときはこの関数を個々のcrazyflieに対し呼び出す必要がある
                for cf in self.allcfs.crazyflies:
                    cf.notifySetpointsStop(100)

                # 着陸, targethightはプロペラ停止高度，durationは着陸にかける時間
                self.allcfs.land(targetHeight=0.02, duration=4.0)
                rospy.sleep(3) # 3秒停止
                break

        print("experiment finish!!")

#####################################################################################################

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

###############################################################################################

if __name__ == '__main__':
    
    # 初期情報の入力
    experiment_time = int(input("実験時間:"))
    hight_start = float(input("init_Z:"))
    # position_destination = list((float(input("dest_X:")), float(input("dest_Y:")), float(input("dest_Z:"))))
    
    # const_value_controlを初期化し，main関数を実行
    coverage_control(experiment_time, hight_start).main()
    # coverage_control(experiment_time).main()