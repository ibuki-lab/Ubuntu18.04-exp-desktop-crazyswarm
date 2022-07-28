#!/usr/bin/env python
# coding: UTF-8

import numpy as np
from turtle_cc_frames_setup import Frames_setup
# 被覆制御用用ライブラリ
from shapely.geometry import Polygon
from scipy.spatial import Voronoi, voronoi_plot_2d
from cvxopt import matrix, solvers
import cvxopt

class Vel_controller(object):

    # 決めた目標値を取得し各パラメータを初期化
    def __init__(self):

        # PIDゲイン, x, y, z, yaw
        self.KP = np.array([.1, .1, .3, .001])
        self.KI = np.array([.0, .0, .0, .0])
        self.KD = np.array([.0, .0, .0, .0])

        # 速度入力, x, y, z, yaw
        self.vel = np.array([.0, .0, .0, .0])

        # 目標値と偏差, x, y, z, yaw
        self.des = np.array([.0, .0, .0, .0])
        self.err = np.array([.0, .0, .0, .0])
        self.err_sum = np.array([.0, .0, .0, .0])
        self.err_prev = np.array([.0, .0, .0, .0])

        # 達成するフォーメーションを構築
        self.consensus_pos = 0.8*np.array([[-1/2.0, -np.sqrt(3.0)/2.0, 0.75, 0.0],
                              [-1/2.0, np.sqrt(3.0)/2.0, 0.75, 0.0],
                              [1/2.0, np.sqrt(3.0)/2, 0.75, 0.0],
                              [-1, 0.0, 0.75, 0.0],
                              [1, 0.0, 0.75, 0.0],
                              [1/2.0, -np.sqrt(3.0)/2.0, 0.75, 0.0]])
        self.consensus_pos3 = np.array([[0.5, 0.5, 0.75, 0],
                                        [1.0, 0.0, 0.75, 0],
                                        [0.0, 0.0, 0.75, 0]])
        # 正三角形のフォーメーション
        self.consensus_equailateral_triangle3 = np.array([[1.0/2, -np.sqrt(3.0)/4, 0.75, 0.0],
                                                          [0.0, np.sqrt(3.0)/4, 0.75, 0.0],
                                                          [-1.0/2, -np.sqrt(3.0)/4, 0.75, 0.0]])

        # ドローンの直径
        self.R = 0.3
        # 計算開始範囲
        self.D = 0.6
        # 二次計画用行列H
        self.H = matrix(np.eye(2).astype(np.float))

    # 速度指令値をPIDコントローラで決定する
    def cmd_controller_output(self, cfs_state_now, yaw_des, L, cur_cf_idx, turtle_pos_X, turtle_pos_Y):

        self.turtle_pos = np.array([[turtle_pos_X, turtle_pos_Y, 0.0, 0.0],
                                    [turtle_pos_X, turtle_pos_Y, 0.0, 0.0],
                                    [turtle_pos_X, turtle_pos_Y, 0.0, 0.0]])

        self.fake_pos = cfs_state_now[:, :] - (self.consensus_equailateral_triangle3 + self.turtle_pos)

        self.err_prev = self.err
        self.err = np.matmul(-L[cur_cf_idx, :], self.fake_pos)
        self.err_sum += self.err

        # print(self.err)

        # PID制御入力を計算
        self.vel = np.multiply(self.KP, self.err) \
                 + np.multiply(self.KI, self.err_sum) \
                 + np.multiply(self.KD, self.err_prev) \
                 + np.array([0.0, 0.0, 0.3 * (1.0 - cfs_state_now[cur_cf_idx, 2]), 0.0])
        
        return self.vel

        
    
    # 衝突回避
    def collision_avoidance(self, cfs_state_now, vel, cur_cf_idx):

        pos_xy = cfs_state_now[cur_cf_idx, :2]
        pos_xy_others = list(np.vstack((cfs_state_now[:cur_cf_idx, :2], cfs_state_now[cur_cf_idx+1:, :2])))

        A = np.array([0, 0])
        b = np.array([0])
        coll=False
        for pos_xy_other in pos_xy_others:
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
            sol = solvers.qp(P=self.H, q=f, G=A, h=b)

            vel[:2] = np.array([sol['x'][0], sol['x'][1]])
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
