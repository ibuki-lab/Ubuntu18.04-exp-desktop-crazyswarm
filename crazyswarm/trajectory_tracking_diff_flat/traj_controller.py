#!/usr/bin/env python
# coding: UTF-8

import math
from matplotlib.pyplot import xkcd
import numpy as np
from  tc_frames_setup import Frames_setup


class Vel_controller(object):

    # 決めた目標値を取得し各パラメータを初期化
    def __init__(self, c, r, T):
        
        # 軌道（円）のパラメータ
        self.c = c
        self.r = r

        # 姿勢のゲイン
        self.KR = 0.3
        #



    # 速度指令値をPIDコントローラで決定する
    def cmd_controller_output(self,
                            R,
                            t,
                            T
                            ):
        
        # 周期を変えたいときはコメントアウト解除
    
        # 軌道設計
        r = self.r
        omega = 2*np.pi/T
        w = omega
        wt = omega * t
        
        ##### circle #####
        # x = self.c + np.array([r*np.sin(wt), r*np.cos(wt), 0])
        # dx = np.array([r*omega*np.cos(wt), -r*omega*np.sin(wt), 0])
        # d2x = np.array([-r*omega**2*np.sin(wt), -r*omega**2*np.cos(wt), 9.8])
        # d3x = np.array([-r*omega**3*np.cos(wt), r*omega**3*np.sin(wt), 0])

        ##### 3D figure 8 #####
        # x = self.c + np.array([r*np.sin(wt), r*np.sin(wt/2), r/4*np.sin(wt/2)])
        # dx = np.array([r*omega*np.cos(wt), r*(omega/2)*np.cos(wt/2), r/4*(omega/2)*np.cos(wt/2)])
        # d2x = np.array([-r*omega**2*np.sin(wt), -r*(omega/2)**2*np.sin(wt/2), 9.81-r/4*(omega/2)**2*np.sin(wt/2)])
        # d3x = np.array([-r*omega**3*np.cos(wt), -r*(omega/2)**3*np.cos(wt/2), -r/4*(omega/2)**3*np.cos(wt/2)])

        #### complex trajectory ####
        A1 = 1.5; A2 = -0.5; A3 = -0.9; A4 = -0.7
        W1 = 0.5 * w; W2 = 1.5 * w; W3 = 0.8 * w; W4 = 1.05 * w

        x = r * np.array([A1 * np.sin(W1 * t) + A2 * np.sin(W2 * t), A3 * np.cos(W3 * t) + A4 * np.cos(W4 * t), 0])
        dx = r*np.array([A1*W1*np.cos(W1*t) + A2*W2*np.cos(W2*t), - A3*W3*np.sin(W3*t) - A4*W4*np.sin(W4*t), 0])
        d2x = r*np.array([-A1*(W1)**2*np.sin(W1*t) - A2*(W2)**2*np.sin(W2*t), - A3*(W3)**2*np.cos(W3*t) - A4*(W4)**2*np.cos(W4*t), 0]) + np.array([00.0, 0.0, 9.8])
        d3x = r*np.array([-A1*(W1)**3*np.cos(W1*t) - A2*(W2)**3*np.cos(W2*t),  A3*(W3)**3*np.sin(W3*t) + A4*(W4)**3*np.sin(W4*t), 0])
    

        yaw = 0
        yaw_rate = 0
        yawS = np.sin(yaw); yawC = np.cos(yaw); 

        # 所望の回転行列の計算

        roll = math.atan( (d2x[0]*yawS - d2x[1]*yawC) / np.sqrt(d2x[2]**2 + (d2x[0]*yawC + d2x[1]*yawS)**2) )
        pitch = math.atan( (d2x[0] * yawC + d2x[1]*yawS) / d2x[2] )

        rollS = np.sin(roll); rollC = np.cos(roll)
        pitchS = np.sin(pitch); pitchC = np.cos(pitch)

        # Rx = np.array([[1, 0,     0    ],
        #                [0, rollC, -rollS],
        #                [0, rollS, rollC]])
        # Ry = np.array([[pitchC,  0, pitchS],
        #                [0,       0, 0     ],
        #                [-pitchS, 0, pitchC]])
        # Rz = np.array([[yawC, -yawS, 0],
        #                [yawS, yawC,  0],
        #                [0,    0,     0]])
        
        # Rd = np.matmul(np.matmul(Rz, Ry), Rx)

        # # 姿勢の偏差を取得
        # Re = 0.5 * (np.matmul(Rd.T, R) - np.matmul(R.T, Rd)) 
        # Re = np.array([Re[2, 1], -Re[2, 0], Re[1, 0]])

        # 所望の（ワールド座標から見た）角速度を取得

        roll_rate_num = d2x[2]**2 * ( d3x[0]*yawS - d3x[1]*yawC + yaw_rate * (d2x[0]*yawC + d2x[1]*yawS) ) - d2x[2]*d3x[2]*(d2x[0]*yawS - d2x[1]*yawC) + (d2x[0]*yawC + d2x[1]*yawS)*(yaw_rate*(d2x[0]**2 + d2x[1]**2) - d2x[0]*d3x[1] + d2x[1]*d3x[0])
        roll_rate_den = (d2x[0]**2 + d2x[1]**2 + d2x[2]**2) * np.sqrt(d2x[2]**2 + ( d2x[0]*yawC + d2x[1]*yawS )**2)
        roll_rate = roll_rate_num / roll_rate_den

        pitch_rate_num = d2x[2]*(d3x[0]*yawC + d3x[1]*yawS) - d3x[2]*(d2x[0]*yawC + d2x[1]*yawS) - d2x[2]*yaw_rate*(d2x[0]*yawS + d2x[1]*yawC)
        pitch_rate_den = d2x[2]**2 + ( d2x[0]*yawC + d2x[1]*yawS )**2
        pitch_rate = pitch_rate_num / pitch_rate_den

        euler_rate = np.array([roll_rate, pitch_rate, yaw_rate])

        w2b = np.array([[1, 0,      -pitchS],
                        [0, rollC,  pitchC * rollS],
                        [0, -rollS, pitchC * rollC]])
        wd = np.matmul(w2b, euler_rate.T)


        # 重力補正なくす
        d2x[2] = 0.0
        # cmdFullState関数を使う場合の入力を計算
        self.Pd = x
        self.Vd = dx
        self.Ad = d2x
        self.yaw = yaw
        self.wd = wd 
        

        return self.Pd, self.Vd, self.Ad, self.yaw, self.wd, roll, pitch
        # 

    """
    # この関数を呼び出すだけで各速度指令値が更新される
    """