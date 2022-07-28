#!/usr/bin/env python
# coding: UTF-8

import numpy as np
from scipy import allclose
from cc_frames_setup import Frames_setup
# 被覆制御用用ライブラリ
from shapely.geometry import Polygon
from scipy.spatial import Voronoi, voronoi_plot_2d
from cvxopt import matrix, solvers
import cvxopt

cvxopt.solvers.options['show_progress'] = False

class Vel_controller(object):

    # 決めた目標値を取得し各パラメータを初期化
    def __init__(self):

        # ゲイン, x, y, z, yaw
        self.K = np.array([.1, .1, .3, .001])

        # 速度入力, x, y, z, yaw
        self.vel = np.array([.0, .0, .0, .0])
        self.ZeroXY = np.zeros((6, 2))
        self.ZeroYaw = np.zeros((6, 1))

        # 目標値と偏差, x, y, z, yaw
        self.des = np.array([.0, .0, .0, .0])
        self.err = np.array([.0, .0, .0, .0])
        self.err_sum = np.array([.0, .0, .0, .0])
        self.err_prev = np.array([.0, .0, .0, .0])

        # 達成するフォーメーションを構築 
        self.consensus_pos = 0.8*np.array([[-1/2.0, -np.sqrt(3.0)/2.0,  0.0,  0.0],
                                           [-1/2.0,  np.sqrt(3.0)/2.0,  0.0,  0.0],
                                           [ 1/2.0,  np.sqrt(3.0)/2.0,  0.0,  0.0],
                                           [-1,      0.0,               0.0,  0.0],
                                           [ 1,      0.0,               0.0,  0.0],
                                           [ 1/2.0,  -np.sqrt(3.0)/2.0, 0.0,  0.0]])

        # self.consensus_pos3 = np.array([[0.5, 0.5, 0.75, 0],
        #                                 [1.0, 0.0, 0.75, 0],
        #                                 [0.0, 0.0, 0.75, 0]])


        # ドローンの直径
        self.R = 0.3
        # 計算開始範囲
        self.D = 0.5
        # 二次計画用行列H
        self.H = matrix(np.eye(2).astype(np.float))

    # 速度指令値をPIDコントローラで決定する
    def cmd_controller_output(self, cfs_state_now, yaw_des, L, flag):
        
        # フォーメーション制御入力を計算
        if flag == "formation":
            self.fake_pos = cfs_state_now[:, :] - self.consensus_pos
            self.err = np.matmul(-L, self.fake_pos)
            self.vel = np.multiply(self.K, self.err) 

        
        # # 高さの合意制御入力を計算
        if flag == "height_consensus":
            self.cfs_hight = cfs_state_now[:, 2]
            self.err = np.matmul(-L, self.cfs_hight)
            Zvel = 0.2 * self.err
            self.vel = np.hstack((np.hstack((self.ZeroXY, np.array([Zvel]).T)), self.ZeroYaw))

        return self.vel

        
    
    # 衝突回避
    def collision_avoidance(self, cfs_state_now, vel, cur_cf_idx):

        pos_xy = cfs_state_now[cur_cf_idx, :2]
        all_pos_xy = cfs_state_now[:, :2]

        A = np.array([0, 0])
        b = np.array([0])
        coll=False
        for j in range(6):
            if j != cur_cf_idx:
                pos_xy_other = all_pos_xy[j, :]
                # 他のエージェントとの相対距離を取得
                L2norm = np.linalg.norm((pos_xy - pos_xy_other), ord=2)
                if L2norm < self.D:
                    coll = True
                    A = np.vstack((A, pos_xy_other - pos_xy))
                    b = np.vstack((b, L2norm**2 - self.R**2))
                    # Ax >= b
        if coll:
            A = matrix(A[1:, :].astype(np.float))
            b = matrix((b[1:, :].astype(np.float))) 

            # xHx' + fx' = 0
            f = -matrix(2*vel[:2].astype(np.float))
            try:
                sol = solvers.qp(P=self.H, q=f, G=A, h=b)
                vel[:2] = np.array([sol['x'][0], sol['x'][1]])
            except:
                vel[:2] = np.array([0.0, 0.0])
                
                print('cf{} : Could not Find Solution !!!', format(cur_cf_idx))

        # print(vel)
        # 衝突回避の制御入力をサチらせる
        vel[0] = min(0.2, max(-0.2, vel[0]))
        vel[1] = min(0.2, max(-0.2, vel[1]))
        
        return vel
    # 実験可能範囲を制限するための関数
    def safty_ctrl(self, cf_state_now, vel):
        vel[:2] = np.multiply(self.KP[0:2], -cf_state_now)

        return vel

    """
    # この関数を呼び出すだけで各速度指令値が更新される
    """
