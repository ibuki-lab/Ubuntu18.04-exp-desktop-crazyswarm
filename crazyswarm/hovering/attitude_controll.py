#!/user/bin/env python
# coding: UTF-8

import sys
import numpy as np
import datetime
import time

import pandas as pd
import matplotlib.pyplot as plt

# from ros_ws.src.crazyswarm.scripts.pycrazyswarm.crazyswarm import Crazyswarm
from pycrazyswarm import *

import rospy
import tf2_ros
import tf_conversions
import tf
from crazyflie_driver.msg  import GenericLogData

# 関連モジュールのインポート
from frames_setup import Frames_setup

# 定値制御
class attitude_control( Frames_setup):

    # このクラスの初期設定を行う関数
    def __init__(self, T):
        # frames_setup, vel_controller の初期化
        super(Frames_setup, self).__init__()

        self.Tend = T
        self.Tsam = 0.01

        self.cmd_sub = rospy.Subscriber("/cf20/log1", GenericLogData, self.log_callback)
        datalen = int(self.Tend/self.Tsam)
        self.cmd_thrust = [0]*datalen
        self.cmd_roll = [0]*datalen
        self.cmd_pitch = [0]*datalen
        self.cmd_yaw = [0]*datalen

        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs
        self.cf = self.allcfs.crazyflies[0]
        
        self.world_frame = Frames_setup().world_frame
        self.child_frame = Frames_setup().children_frame[0]
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        
        time.sleep(0.5)

        self.zero3d = np.array([0, 0, 0])
    def log_callback(self, log):
        self.Log = log.values

    def get_state(self):
        try:
            f = self.tfBuffer.lookup_transform(self.world_frame, self.child_frame, rospy.Time(0))

        # 取得できなかった場合は0.5秒間処理を停止し処理を再開する
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('LookupTransform Error !')
            rospy.sleep(0.5)
            exit()
        return f

    def get_log(self, cnt):
        print(self.Log)
        self.cmd_thrust[cnt] = self.Log[0]
        self.cmd_roll[cnt] = self.Log[1]
        self.cmd_pitch[cnt] = self.Log[2]
        self.cmd_yaw[cnt] = self.Log[3]
        
    def save_log(self):
        data = {"input thrust": self.cmd_thrust}
        data = {"input yaw": self.cmd_roll}
        data = {"input pitch": self.cmd_pitch}
        data = {"input yaw": self.cmd_yaw}
        df = pd.DataFrame(data)
        df.to_csv("thrust_data_log{}".format(datetime.date.today()))

    def time_check(self, t, Tint):
        if Tint < self.Tsam:
            time.sleep(self.Tsam - Tint)
        if t > self.Tend:
            return True
        return False

    def main(self):

        # 実験開始
        print("Experiment Start!!")
        time.sleep(1)
        cnt = 0
        Ts = time.time()
        while True:
            Tint = time.time()
            t = time.time() - Ts
            f = self.get_state()

            self.cf.cmdFullState(pos=self.zero3d, vel=self.zero3d, acc=np.array([0,0,1]), yaw=0.0, omega=self.zero3d)
            self.get_log(cnt)

            if self.time_check(t, time.time() - Tint):
                break
            cnt += 1
            
        # save log data
        self.save_log()


if __name__ == '__main__':
    T = float(input("exp time"))
    attitude_control(T).main()
    exit()