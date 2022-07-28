#!/usr/bin/env python
# coding: UTF-8

from cvxopt import matrix, solvers
import cvxopt
import numpy as np


cvxopt.solvers.options['show_progress'] = False
class vel_controller_coll_avoid(object):

    def __init__(self, pos_des, alpha):
        
        # PIDゲイン
        self.Xp = 0.2
        self.Xi = 0.00001
        self.Xd = 0.00

        self.Yp = 0.2
        self.Yi = 0.00001
        self.Yd = 0.00

        self.Zp = 0.3
        self.Zi = 0.0
        self.Zd = 0.0

        self.yawp = 0.01
        self.yawi = 0.0
        self.yawd = 0.0

        # 速度入力
        self.X_dot = 0.0
        self.Y_dot = 0.0
        self.Z_dot = 0.0
        self.yaw_dot = 0.0

        # 目標値と偏差
        self.X_des = pos_des[0]
        self.X_err = 0.0
        self.X_err_prev = 0.0
        self.X_err_sum = 0.0

        self.Y_des = pos_des[1]
        self.Y_err = 0.0
        self.Y_err_prev = 0.0
        self.Y_err_sum = 0.0

        self.Z_des = pos_des[2]
        self.Z_err = 0.0
        self.Z_err_prev = 0.0
        self.Z_err_sum = 0.0

        self.yaw_des = 0.0
        self.yaw_err = 0.0
        self.yaw_err_prev = 0.0
        self.yaw_err_sum = 0.0

        self.A = 0.0
        self.f = 0.0
        self.b = 0.0
        self.h = 0.0

        # 二次計画法のハイパーパラメータ
        self.alpha = alpha
    
    def cmd_controller_collision_avoidance_output(self, pos_now, pos_des, yaw_now, yaw_des, pos_obs, r, dt):

        self.X_err_prev = self.X_err
        self.X_err = pos_des[0] - pos_now[0]
        self.X_err_sum += self.X_err

        self.X_dot = self.Xp*self.X_err + self.Xi*self.X_err_sum + self.Xd*(self.X_err - self.X_err_prev) / (dt*self.num_cf)

        self.Y_err_prev = self.Y_err
        self.Y_err = pos_des[1] - pos_now[1]
        self.Y_err_sum += self.Y_err

        self.Y_dot = self.Yp*self.Y_err + self.Yi*self.Y_err_sum + self.Yd*(self.Y_err - self.Y_err_prev) / (dt*self.num_cf)

        self.Z_err_prev = self.Z_err
        self.Z_err = pos_des[2] - pos_now[2]
        self.Z_err_sum += self.Z_err

        self.Z_dot = self.Zp*self.Z_err + self.Zi*self.Z_err_sum + self.Zd*(self.Z_err - self.Z_err_prev) / (dt*self.num_cf)

        self.yaw_err_prev = self.yaw_err
        self.yaw_err = yaw_des - yaw_now
        self.yaw_err_sum += self.yaw_err

        self.yaw_dot = self.yawp*self.yaw_err + self.yawi*self.yaw_err_sum + self.yawd*(self.yaw_err - self.yaw_err_prev) / (dt*self.num_cf)


        # 二次計画法
        # 現在の位置ベクトル
        self.posXY = np.array([pos_now[0], pos_now[1]])
        # Ax <= b
        self.A = matrix((pos_obs - self.posXY).astype(np.float)).T
        self.b = matrix( self.alpha * (((np.linalg.norm((self.posXY - pos_obs), ord=2)**2) - r**2).astype(np.float))/2)
        
        # xhx' + fx' = 0
        self.h = matrix(np.array([[1.0, 0.0], [0.0, 1.0]]).astype(np.float))/2
        self.f = - matrix(np.array([self.X_dot, self.Y_dot]).astype(np.float))

        sol = solvers.qp(P=self.h, q=self.f, G=self.A, h=self.b)
        
        self.X_dot = sol['x'][0]
        self.Y_dot = sol['x'][1]





