#!/user/bin/env python
# coding: UTF-8

##### 一番最初に作られたと思われる被覆制御のプログラミング #####

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

############## 被覆制御 ##############################################################################

class coverage_control( Frames_setup, Coverage_controller):

##### このクラスの初期設定を行う関数 ##############################################################
    
    def __init__(self, experiment_time, hight_start):

        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__()

    # 実行時間, 初期高度, 目標位置, 位置記録用リストの用意
        self.exp_time = experiment_time
        self.hight_start = hight_start
        #self.pos_des = pos_des
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
        
    
    # tfフレームの初期化，フレーム間(原点と個々のcrazyflie)の位置関係が取得可能になる
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
    
    # crazyflieの個体数も記録
        self.num_cf = len(self.child_frames)

#####################################################################################

#######################################################################################
    def get_voronoi(self,bnd, pnts):
        # bnd：ボロノイ分割する領域 範囲
        # pnts:母点座標
        """
        有界なボロノイ図を計算・描画する関数
        """

        # すべての母点のボロノイ領域を誘拐にするために，ダミー母点を三個追加
        # concatenate: 行列を縦方向に結合する
        gn_pnts = np.concatenate([pnts, np.array([[100, 100], [100, -100], [-100, 0]])])

        # ボロノイ図の計算
        vor = Voronoi(gn_pnts)

        # 分割する領域をPolygonにする，有界領域
        bnd_poly = Polygon(bnd)
        # 各ボロノイ領域をしまうリスト
        vor_polys = []
        # 各ボロノイ領域のpolygonの頂点座標を格納
        poly_vertice = []
        # 各ボロノイ領域の平面座標系の頂点座標を格納
        pos_vertice = []

        for i in range(len(gn_pnts) - 3):
            
            # 平空間を考慮しないボロノイ領域
            """
            vertices: ボロノイ頂点座標を返す
            regions:すべてのボロノイ領域を格納している，
            region:ボロノイ領域
            point_region:ボロノイ領域の点が格納されている

            結局ボロノイ領域の各頂点座標を取得している
            """
            vor_poly = [vor.vertices[v] for v in vor.regions[vor.point_region[i]]]


            # 分割する領域をボロノイ領域の共通部分を計算
            """
            intersection: 積集合
            重複なしのボロノ頂点を取得
            """
            i_cell = bnd_poly.intersection(Polygon(vor_poly))
            pos_vertice.append(i_cell.exterior.coords[:-1])
            poly_vertice.append(Polygon(i_cell.exterior.coords[:-1]))

            # 平空間を考慮したボロノイ領域の頂点座標を格納
            vor_polys.append(list(i_cell.exterior.coords[:-1]))

        # 各母点の位置，ボロノイ図の領域，ボロノイ図の各頂点，ボロノイ図の各頂点(座標平面に適応済み)
        return gn_pnts, vor_polys, poly_vertice
###############################################################################################################

    def get_centroid(poly_gem):

        # assert type(gem) == "shapely", "input should be a Shapely geometory"
        # assert gem.geom_type in ['Point', 'LineSrtring', 'Polygon'], "Input should be a Shapely geometry"
        # ポリゴンに変換
        # 重心を求める

        centroid = []
        for i in range(len(list(poly_gem))):
            centroid.append(poly_gem[i].centroid)
        
        return centroid

#################################################################################################################        

    # 求めた重心との偏差を計算する
    def get_err(pnts, centroid):
        x_err = []
        y_err = []
        for i in range(len(pnts)):
            x_err.append(pnts[i][0] - np.array(centroid[i])[0] + 0.0001 * np.random.randn())
            y_err.append(pnts[i][1] - np.array(centroid[i])[1] + 0.0001 * np.random.randn())
        return x_err, y_err

    # 求めた偏差から一度に動く量を計算する
    def get_move(x_err, y_err):
        gain = 0.1
        x_move = []
        y_move = []
        for i in range(len(x_err)):
            x_move.append(-gain*x_err[i] + 0.00* np.random.randn())
            y_move.append(-gain*y_err[i] + 0.00 * np.random.randn())
        return x_move, y_move 

    # 動いた後の位置を更新
    def get_newpnts(pnts, x_move, y_move):
        for i in range(len(pnts)):
            pnts[i][0] += x_move[i]
            pnts[i][1] += y_move[i]
        return pnts

    # ボロノイ図の描画
    def plot_figure(bnd, gn_pnts, vor_polys, fig, ax):
        
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
        """
        verts: 各ポリゴンの頂点を二次配列で渡す[[2, 2], [2, 3]]みたいに
        """
        poly_vor = PolyCollection(verts=vor_polys, edgecolor="black", facecolors="None", linewidth=1.0)
        ax.add_collection(poly_vor)
        fig.canvas.draw()

#######################################################################################################

    # 各エージェントに指令値を送る関数
    def main(self):

        # まず離陸させる，高さはclass呼び出し時に設定
        self.allcfs.takeoff(targetHeight=self.hight_start, duration=2.0)
        rospy.sleep(3)  # 3秒静止

        # 実験開始
        print("Experiment Start!!")
        start_time = time.time() # 開始時刻の取得，実験時間を図るため
        dt=0.001
        
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(111)
        plt.ion()
        fig.show()
        fig.canvas.draw()

        # ボロノイ分割する領域
        bnd = np.array([[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5]])

        # 母点の個数
        n = self.num_cf
        print(n)
        
        while True:
            # 各crazyflieに指令を送る!子フレーム(cf?)と命令対象のcrazyflieが一致している事が絶対条件 !
            for cf, child_frame in zip(self.allcfs.crazyflies, self.child_frames):
                
                # 原点と一つのcrazyflieとの座標変換を取得
                try:
                    t = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))

                # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr('LookupTransform Error !')
                    rospy.sleep(0.5)
                    continue   
            
            # クオータニオン(四元数)の取得
                self.Quaternion = (t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
            # オイラー角の取得
                self.RPY = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)
            # 同次座標の取得
                self.homogerous_matrixs = tf_conversions.transformations.quaternion_matrix(self.Quaternion) + np.array([[0, 0, 0, t.transform.translation.x],
                                                                                                                        [0, 0, 0, t.transform.translation.y],
                                                                                                                        [0, 0, 0, t.transform.translation.z],
                                                                                                                        [0, 0, 0, 0]])

######## ボロノイ分割をして重心（目標値を求める） #######################################################################################################################################################################################################################################################################                                                                                                                        [0, 0, 0, t.transform.translation.y],

                pnts = self.homogerous_matrixs[:2, 3]
                print(pnts)

            # ボロノイ図を計算
                
                # ボロノイ図の計算・描画        
                gn_pnts, vor_polys, poly_vertice, pos_vertice  = self.get_voronoi(bnd, pnts)
                
                # グラフの描画
                self.plot_figure(bnd, gn_pnts, vor_polys, fig, ax)
                
                # 重心を求める
                centroid = self.get_centroid(poly_vertice)

                # # 重心との偏差を計算する
                x_err, y_err = self.get_err(pnts, centroid)

                # # 偏差から一度に動く量を計算する
                x_move, y_move = self.get_move(x_err,y_err)

                # # 動いた後の位置を更新
                new_pnts = self.get_newpnts(pnts, x_move, y_move)

                # 目標値
                pos_desired = new_pnts.append(hight_start)
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
    coverage_control(experiment_time, hight_start ).main()