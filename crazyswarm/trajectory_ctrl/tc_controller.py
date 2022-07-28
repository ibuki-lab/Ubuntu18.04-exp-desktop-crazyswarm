#!/usr/bin/env python
# coding: UTF-8

class Vel_controller(object):

    # 決めた目標値を取得し各パラメータを初期化
    def __init__(self, trajectory_gain):
        

        # PIDゲイン
        self.Xp = 0.1
        self.Xi = 0.0
        self.Xd = 0.0

        self.Yp = 0.1
        self.Yi = 0.0
        self.Yd = 0.0

        self.Zp = 0.1
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
        self.X_des = 0.0
        self.X_err = 0.0
        self.X_err_prev = 0.0
        self.X_err_sum = 0.0

        self.Y_des = 0.0
        self.Y_err = 0.0
        self.Y_err_prev = 0.0
        self.Y_err_sum = 0.0

        self.Z_des = 0.0
        self.Z_err = 0.0
        self.Z_err_prev = 0.0
        self.Z_err_sum = 0.0

        self.yaw_des = 0.0
        self.yaw_err = 0.0
        self.yaw_err_prev = 0.0
        self.yaw_err_sum = 0.0

        self.trajectory_gain = trajectory_gain

    
    # 速度指令値をPIDコントローラで決定する
    def cmd_controller_output(self, pos_now, yaw_now, pos_des, yaw_des, vel_tra):
        
    # X_dotの計算
        self.X_dot = self.trajectory_gain*(pos_des[0] - pos_now[0]) + vel_tra[0]

    # Y_dotの計算
        self.Y_dot = self.trajectory_gain*(pos_des[1] - pos_now[1]) + vel_tra[1]

    # Z_dotの計算
        self.Z_dot = self.Zp * (pos_des[2] - pos_now[2])

    # yaw_dotの計算
        self.yaw_dot = self.yawp * (yaw_des - yaw_now)
    """
    # この関数を呼び出すだけで各速度指令値が更新される
    """