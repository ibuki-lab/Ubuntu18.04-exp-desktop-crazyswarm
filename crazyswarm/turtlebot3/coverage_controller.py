#!/usr/bin/env python
# coding: UTF-8

class Coverage_controller(object):

    # 決めた目標値を取得し各パラメータを初期化
    def __init__(self):
        

        # PIDゲイン
        self.Xp = 0.6
        self.Xi = 0.00001
        self.Xd = 0.000

        self.Yp = 0.6
        self.Yi = 0.00001
        self.Yd = 0.0000

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
    
    # 速度指令値をPIDコントローラで決定する
    def cmd_controller_output(self, pos_now, pos_des, yaw_now, yaw_des, dt):
        
    # X_dotの計算
        self.X_err_prev = self.X_err                  # 比例項
        self.X_err = pos_des[0] - pos_now[0]          # 微分項
        self.X_err_sum += self.X_err                  # 積分項

        self.X_dot = self.Xp*self.X_err + self.Xi*self.X_err_sum + self.Xd*(self.X_err - self.X_err_prev) / (dt*self.num_cf)
    # Y_dotの計算
        self.Y_err_prev = self.Y_err
        self.Y_err = pos_des[1] - pos_now[1]
        self.Y_err_sum += self.Y_err

        self.Y_dot = self.Yp*self.Y_err + self.Yi*self.Y_err_sum + self.Yd*(self.Y_err - self.Y_err_prev) / (dt*self.num_cf)
    # Z_dotの計算
        self.Z_err_prev = self.Z_err
        self.Z_err = pos_des[2] - pos_now[2]
        self.Z_err_sum += self.Z_err

        self.Z_dot = self.Zp*self.Z_err + self.Zi*self.Z_err_sum + self.Zd*(self.Z_err - self.Z_err_prev) / (dt*self.num_cf)
    # yaw_dotの計算
        self.yaw_err_prev = self.yaw_err
        self.yaw_err = yaw_des - yaw_now
        self.yaw_err_sum += self.yaw_err

        self.yaw_dot = self.yawp*self.yaw_err + self.yawi*self.yaw_err_sum + self.yawd*(self.yaw_err - self.yaw_err_prev) / (dt*self.num_cf)

    # この関数を呼び出すだけで各速度指令値が更新される