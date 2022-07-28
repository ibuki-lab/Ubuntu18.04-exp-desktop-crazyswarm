#!/user/bin/env python
# coding: UTF-8
# from ros_ws.src.crazyswarm.scripts.pycrazyswarm.crazyswarm import Crazyswarm
import numpy as np
import rospy
import tf2_ros
from tf2_ros import transform_broadcaster
import tf_conversions
# FullState: geometry_msg/Pose, geometry_msg/Twist, geometry_msg/Vector3
#from crazyflie_driver import FullState
# Quantanion用topic
from geometry_msgs.msg import TransformStamped, Transform, Twist, TwistStamped, Vector3, Quaternion
# 位置姿勢topic
from tf2_msgs.msg import TFMessage

import tf
from pycrazyswarm import *
import time

if __name__ == '__main__':
    rospy.init_node('tf2_listener')

    # listenerのインスタンス化、この時点から配信されるフレームの受信が始まる
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    r = rospy.Rate(1)
    r.sleep()
    while not rospy.is_shutdown():
        # 親フレームと子フレームの定義
        world_frame = 'world'
        chiled_frame = 'cf5'
        try:
            # 2フレーム間の座標変換を取得
            t = tfBuffer.lookup_transform(world_frame, chiled_frame, rospy.Time(0))
        # 取得できなかった場合は１秒間処理を停止し再び再開する
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('LookupTransform Error !')
            rospy.sleep(1.0)
            continue
        # 相対位置、姿勢を取得
        # translation = (t.transform.translation.x, 
        #                t.transform.translation.y,
        #                t.transform.translation.z)

        rotation = (t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w)
        # オイラー角に変換
        print(type(t.transform.rotation))
        homogerous_matrixs = tf_conversions.transformations.quaternion_matrix((t.transform.rotation.x,
                                                                               t.transform.rotation.y,
                                                                               t.transform.rotation.z,
                                                                               t.transform.rotation.w)) \
                                                                                   + np.array([[0, 0, 0, t.transform.translation.x],
                                                                                               [0, 0, 0, t.transform.translation.y],
                                                                                               [0, 0, 0, t.transform.translation.z],
                                                                                               [0, 0, 0, 0]])
        print(homogerous_matrixs)

        # rospy.loginfo("\n=== Got Transform ===\n"
        #               "Traanslation\n"
        #               "x : %f\n y : %f\n z : %f\n"
        #               "Quaternion"
        #               "x : %f\n y : %f\n z : %f\n w : %f\n"
        #               "RPY\n"
        #               "R : %f\n P : %f\n Y : %f\n",
        #               translation[0], translation[1], translation[2],
        #               rotation[0], rotation[1], rotation[2], rotation[3],
        #               roll, pitch, yaw)
        time.sleep(0.01)
