#!/usr/bin/env python
# coding: UTF-8

import numpy as np
from  drones_coll_avoid_frames_setup import Frames_setup
from cvxopt import matrix, solvers
import cvxopt

cvxopt.solvers.options['show_progress'] = False

class Vel_controller(object):

    # 決めた目標値を取得し各パラメータを初期化
    def __init__(self, alpha):
        

        # PIDゲイン, x, y, z, yaw
        self.KP = np.array([.5, .5, .5, .001])
        self.KI = np.array([.0, .0, .0, .0])
        self.KD = np.array([.0, .0, .0, .0])

        # 速度入力, x, y, z, yaw
        self.vel = np.array([.0, .0, .0, .0])

        # 目標値と偏差, x, y, z, yaw
        self.des = np.array([.0, .0, .0, .0])
        self.err = np.array([.0, .0, .0, .0])
        self.err_sum = np.array([.0, .0, .0, .0])
        self.err_prev = np.array([.0, .0, .0, .0])

        # 回避する程度
        self.alpha = 0.1
        # ドローンの大きさを定義
        self.drone_R = 0.2
        # 衝突回避計算開始範囲
        self.drone_Da = 0.4
        #　cfsの初期位置を取得
        world_frame = Frames_setup().world_frame
        child_frames = Frames_setup().children_frame
        self.cfs_num = len(child_frames)

        self.H = matrix(np.eye(3).astype(float))
    # 速度指令値をPIDコントローラで決定する
    def cmd_controller_output(self,
                                i, 
                                des,
                                g,
                            ):
        flag = False
        e = des - g[i][:3, 3]
        self.vel = np.multiply(self.KP[:3], e)

        self.A = np.array([0, 0, 0])
        self.b = np.array([0])
        self.f = -matrix(2*self.vel.astype(float))
        
        for j in range(len(g)):
            g_ij = np.matmul(np.linalg.inv(g[i]), g[j])
            diff_ij = g_ij[0, 3]**2/2 + g_ij[1, 3]**2/2 + g_ij[2, 3]**2/5
            if diff_ij < self.drone_Da**2 and i != j:
                self.A  = np.vstack((self.A, g_ij[0:3, 3]))
                self.b = np.vstack((self.b, self.alpha * (diff_ij - self.drone_R**2)/4))
                flag = True
        
        if flag:
            self.A = matrix(self.A[1:, :].astype(float))
            self.b = matrix(self.b[1:, :].astype(float))

            sol = solvers.qp(P=self.H, q=self.f, G=self.A, h=self.b)

            self.vel = np.array([sol['x'][0], sol['x'][1], sol['x'][2]])
        

    """
    # この関数を呼び出すだけで各速度指令値が更新される
    """