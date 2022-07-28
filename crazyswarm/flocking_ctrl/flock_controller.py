#!/usr/bin/env python
# coding: UTF-8

import numpy as np
import rospy
import tf2_ros
import tf_conversions
import tf


# 被覆制御用用ライブラリ
from shapely.geometry import Polygon
from scipy.spatial import Voronoi, voronoi_plot_2d
from cvxopt import matrix, solvers
import cvxopt

cvxopt.solvers.options['show_progress'] = False

class Vel_controller(object):

    # 決めた目標値を取得し各パラメータを初期化
    def __init__(self):
        

        # PIDゲイン, x, y, z, yaw
        self.KP = np.array([.01, .01, .4, .001])
        self.KI = np.array([.0, .0, .0, .0])
        self.KD = np.array([.0, .0, .0, .0])

        # 速度入力, x, y, z, yaw
        self.vel = np.array([.0, .0, .0, .0])

        # 目標値と偏差, x, y, z, yaw
        self.des = np.array([.0, .0, .0, .0])
        self.err = np.array([.0, .0, .0, .0])
        self.err_sum = np.array([.0, .0, .0, .0])
        self.err_prev = np.array([.0, .0, .0, .0])


        # 衝突回避用パラメータ
        self.H = matrix(np.eye(4).astype(float))
        self.f = matrix(np.zeros((4, 1)).astype(float))
        # 群れ制御最適化用パラメータ
        self.deltamax = np.linalg.norm(np.array([0.31451, 0.13817, 0.17819]), ord=2)
        self.Kp = 0.015
        self.Ke = 1
        self.Kc = 2
        self.Kz = 0.5
        self.Z_bound = 0.9
        self.Dc = 0.2
        self.Da = 0.4
        
    # 速度指令値をPIDコントローラで決定する
    def cmd_controller_output(self,
                            i,
                            G,
                            g,
                            yaw,
                            xy_des
                            ):
        # 最適化用行列
        A = np.array([0, 0, 0, 0])
        b = np.array([0])


        p_sum = 0
        theta_sum = 0
        for j in range(len(g)):
            
            # 相対同次座標を取得
            
            g_ij = np.matmul(np.linalg.inv(g[i]), g[j]) 

            # 衝突回避
            diff_ij = np.linalg.norm(g_ij[:3, 3], ord=2)
            if diff_ij < self.Da and i != j:
                A = np.vstack((A, np.array([g_ij[0, 3], g_ij[1, 3], g_ij[2, 3], 0])))
                # 衝突範囲は球状
                # b = np.vstack((b, self.Kc * (diff_ij**2 - self.Dc**2))) # cbf
                b = np.vstack((b, self.Kc * (diff_ij**2 - self.Dc**2) - diff_ij * self.deltamax)) # ecbf
                # 衝突範囲は楕円状
                #diff_ij = (g_ij[0, 3]/1)**2 + (g_ij[1, 3]/1)**2 + (g_ij[2, 3]/1)**2
                #b = np.vstack((b, self.Kc * (diff_ij - self.Dc**2)))


            # 位置合意と姿勢合意
            if G[i, j] == 1:
                p_sum += g_ij[:3, 3]
                theta_sum += np.sin(yaw[j] - yaw[i])
        
        # 位置制御
        #p_sum += np.hstack((xy_des - g[i][:2, 3], 0))

        A = np.vstack((A, -np.hstack(((p_sum, np.array([1]))))))[1:, :]
        b = np.vstack((b, -self.Kp * np.linalg.norm(p_sum, ord=2)**2))[1:, :]

        # 高度維持
        z = g[i][2, 3]
        A = np.vstack((A, -np.array([0.0, 0.0, z, 0.0])))
        b = np.vstack((b, -self.Kz * (self.Z_bound**2 - z**2)/2))

        A = matrix(A.astype(float))
        b = matrix(b.astype(float))
        
        try:
            # Qpを解く
            sol = solvers.qp(P=self.H, q=self.f, G=A, h=b)

            vel = np.array([sol['x'][0], sol['x'][1], sol['x'][2]])
        except:
            vel = np.array([0.0, 0.0, 0.0])
            
            print('cf{} : Could not Find Solution !!!', format(i))

        yaw_rate = self.Ke * theta_sum

        return vel, yaw_rate

    def safty_ctrl(self, cf_state_now, vel):
        vel[:2] = np.multiply(self.KP[:2], -cf_state_now)
        return vel
    
    def safty_ctrl_Z(self, cf_state_now, vel):
        vel[2] = np.multiply(self.KP[2], max(0.8, min(1.3, cf_state_now)) - cf_state_now)
        return vel