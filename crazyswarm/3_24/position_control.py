#!/user/bin/env python
# coding: UTF-8

from re import T
import numpy as np
import time
import matplotlib.pyplot as plt
import pandas as pd
import datetime

from pycrazyswarm import *

import rospy
import tf2_ros
import tf_conversions
import tf

from vel_controller import Vel_controller
from frames_setup import Frames_setup

class const_value_control( Frames_setup, Vel_controller):

    # このクラスの初期設定を行う関数
    def __init__(self):


        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs

        self.world_frame = Frames_setup().world_frame
        self.child_frames = Frames_setup().children_frame
        self.num_cf = len(self.child_frames) 

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        """--------- 変えてみよう ---------------"""
        self.A = 3
        self.T = 1

        """--------------------------------------"""

        self.goal = 0.0
    
    def main(self):
        cf = self.allcfs.crazyflies[0]
        child_frame = self.child_frames[0]  
        
        cf.takeoff(targetHeight=1.0, duration=5.0)
        rospy.sleep(5)  # 5秒静止

        print('Experiment Start')

        flag = True
        Ts = time.time()
        Tsample = 0.01
        ts = -self.T
        while True:
            t = time.time()-Ts
            roopTs = time.time()
            try:
                f = self.tfBuffer.lookup_transform(self.world_frame, child_frame, rospy.Time(0))
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Error !')
                rospy.sleep(0.5)
                continue      
            
            x = f.transform.translation.x
            z = f.transform.translation.z
            if abs(x) > 1.5:
                flag = False
                print("NoWay")
        
            # 速度入力
            if time.time() - ts > self.T:
                if flag:
                    v = np.array([self.A * self.T * (self.goal - x), 0, 0.5*(1 - z)])
                else:
                    v = np.array([0.5* (0.0 - x), 0, 0.4*(1 - z)])
                ts = time.time()

            cf.cmdVelocityWorld(v, 0)

            
            roopT = time.time()-roopTs
            if roopT < Tsample:
                time.sleep(self.T - roopT)
            
            if t > 20:
                break
        
        cf.notifySetpointsStop(100)
        self.allcfs.land(targetHeight=0.02, duration=4.0)
        rospy.sleep(5) # 5秒停止
if __name__ == '__main__':
    const_value_control().main()
    exit()