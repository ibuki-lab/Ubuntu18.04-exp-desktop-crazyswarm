#!/usr/bin/env python
# coding: UTF-8

import numpy as np
from  drones_coll_avoid_frames_setup import Frames_setup
from cvxopt import matrix, solvers
import cvxopt

cvxopt.solvers.options['show_progress'] = False

class Vel_controller(object):

    # 決めた目標値を取得し各パラメータを初期化
    def __init__(self, alpha, hight):
        

        # PIDゲイン, x, y, z, yaw
        self.KP = np.array([.2, .2, .2, .001])
        self.KP_diff = [np.array([np.random.rand()/10, np.random.rand()/10, 0, 0]) for _ in range(5)]
        self.KI = np.array([.0, .0, .0, .0])
        self.KD = np.array([.0, .0, .0, .0])

        # 速度入力, x, y, z, yaw
        self.vel = np.array([.0, .0, .0, .0])

        # 目標値と偏差, x, y, z, yaw
        self.hight = hight
        self.err = np.array([.0, .0, .0, .0])
        self.err_sum = np.array([.0, .0, .0, .0])
        self.err_prev = np.array([.0, .0, .0, .0])

        # 回避する程度
        self.alpha = alpha

        # ドローンの大きさを定義
        self.drone_R = 0.25

        # ドローンが近傍と判断する距離を設定 40cm
        self.D = 0.4

        #　cfsの初期位置を取得
        world_frame = Frames_setup().world_frame
        child_frames = Frames_setup().children_frame
        self.cfs_num = len(child_frames)

        # 各ドローンの目的地を決めるポイントのカウント
        self.way_point_count = [0, 0, 0, 0, 0]
    
    # 速度指令値をPIDコントローラで決定する
    def cmd_controller_output(self,
                            cfs_pos_now,
                            pos_des,
                            yaw_des, 
                            cur_cf_idx
                            ):

        # このドローンが移動するルートの取得
        # print(cur_cf_idx)
        way_point_num = self.way_point_count[cur_cf_idx]
        # 各軸に対する偏差を取得、行列の積
        self.err_prev = self.err # だめ
        self.err[:2] = pos_des[way_point_num] - cfs_pos_now[cur_cf_idx, :2]
        self.err[2] = self.hight - cfs_pos_now[cur_cf_idx, 2]
        self.err_sum += self.err # だめ
        
        # 目的地にある程度近づいたら目的地を更新　15cm
        if np.linalg.norm((self.err[:2]), ord=2) < 0.15 and len(pos_des) == 3:
            self.way_point_count[cur_cf_idx] = (self.way_point_count[cur_cf_idx] + 1) % 3
            self.KP_diff[cur_cf_idx] = np.array([np.random.rand()/10, np.random.rand()/10, 0, 0])
        elif np.linalg.norm((self.err[:2]), ord=2) < 0.15 and len(pos_des) == 4:
            self.way_point_count[cur_cf_idx] = (self.way_point_count[cur_cf_idx] + 1) % 4
        # PID制御入力を計算
        self.vel = np.multiply(self.KP + self.KP_diff[cur_cf_idx], self.err) \
                 + np.multiply(self.KI, self.err_sum) \
                 + np.multiply(self.KD, self.err_prev) \
                 + np.array([0.0, 0.0, 0.02, .0]) 
                 # multiply:要素ごとの積
        
                  
        # 近傍のドローンの位置情報を取得 
        pos_me = cfs_pos_now[cur_cf_idx, :2]      # 制御対象のドローンのx-y位置座標
        pos_others = []
        for i in range(self.cfs_num):
            if 0 < np.linalg.norm((cfs_pos_now[i, :2] - pos_me), 2) < self.D:
                pos_others.append(cfs_pos_now[i, :2])
        
        # 衝突回避　二次計画法を解く
        H = matrix(np.array([[1.0, 0.0], [0.0, 1.0]]).astype(np.float))
        for pos_other in pos_others:
            f = -matrix(2*self.vel[:2].astype(np.float))
            A = -matrix(np.array([pos_me - pos_other]).astype(np.float))
            b = matrix(self.alpha * (np.linalg.norm((pos_me - pos_other), ord=2)**2 - self.drone_R**2) / 4)
            sol = solvers.qp(P=H, q=f, G=A, h=b)
            self.vel[:2] = np.array([sol['x'][0], sol['x'][1]])
        

        print(self.vel)

    """
    # この関数を呼び出すだけで各速度指令値が更新される
    """