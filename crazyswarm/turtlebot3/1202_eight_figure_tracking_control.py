#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, sqrt, degrees
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

speed = Twist()
l = 0.3
kp = 1
r = 1
omega = 0.15

def callback(data):
    global speed, rate2, l, kp, r, omega, time, pre_u1, pre_u2, post_u1, post_u2
    e = tf.transformations.euler_from_quaternion((data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))
    xt = data.pose.position.x
    yt = data.pose.position.y
    thetat =  e[2]
    x = []
    y = []
    theta = []
    x.append(xt)
    y.append(yt)
    theta.append(thetat)

    # 前方位置の確保
    xc = xt + l * cos(thetat)
    yc = yt + l * sin(thetat)
    st_vec = np.array([[x[0] + l * cos(theta[0])], [y[0] + l * sin(theta[0])] , [theta[0]]])
    
    # 所望の座標位置に、八の字の動きを足している
    go_vec2 = np.array([[0], [0]]) + np.array([[r * sin(omega * time)], [r * sin(2 * omega * time)]])
    d = np.array([[r * omega * cos(omega * time)], [2 * r * omega * cos(2 * omega * time)]])

    # 所望の位置と現在位置の相対となる
    e1 = (go_vec2[0, 0] - xc)
    e2 = (go_vec2[1, 0] - yc)
    # 定常偏差(オフセット)をなくすには、微分したものを足す必要がある
    u1 = cos(thetat) * (kp * e1 + d[0, 0]) + sin(thetat) * (kp * e2 + d[1, 0]) # [m/s]
    u2 = -sin(thetat) * (kp * e1 + d[0, 0]) * (1/l) + cos(thetat) * (kp * e2 + d[1, 0]) * (1/l) # [rad/s]

    pre_u1.append(u1)
    pre_u2.append(u2)

    max_u1 = 0.22 # [m/s]
    min_u1 = -0.22 # [m/s]
    max_u2 = 2.84 # [rad/s]
    min_u2 = -2.84 # [rad/s]

    if u1 > max_u1:
        u1 = max_u1
    if u1 < min_u1:
        u1 = min_u1
    if u2 > max_u2:
        u2 = max_u2
    if u2 < min_u2:
        u2 = min_u2

    post_u1.append(u1)
    post_u2.append(u2)

    speed.linear.x = u1
    speed.angular.z = u2


def tracking_control():
    global speed, rate2, time, pre_u1, pre_u2, post_u1, post_u2
    
    time = 0
    rospy.init_node('tracking_control')
    pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
    rate = rospy.Rate(100)
    rate2 = 1 / 100
    rospy.Subscriber("/mocap_node/turtlebot3/pose", PoseStamped, callback)

    pre_u1 = []
    pre_u2 = []
    post_u1 = []
    post_u2 = []

    while not rospy.is_shutdown():
        pub.publish(speed)
        print(time)
        time = time + 0.01
        rate.sleep()

    plt.plot(pre_u1, label="pre_v")
    plt.plot(pre_u2, label="pre_omega")
    plt.plot(post_u1, label="post_v")
    plt.plot(post_u2, label="post_omega")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    tracking_control()

"""
〜通信方法を確立する〜
$ ssh ubuntu@192.168.0.166(burger)/192.168.0.230(waffle)
PW：turtlebot
$ sudo nano ~/.bashrcで通信するためのRoscoreのIPアドレスを設定する
デスクトップのパソコンなら
$ export ROS_MASTER_URI=http://192.168.0.225:11311
ノートパソコンなら
$ export ROS_MASTER_URI=http://192.168.0.152:11311
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch

〜Optitrack〜
View → Assets Pane → turtlebot3を右クリック → Streaming IDを2に変更する
View → Data Streaming Pane → Up AxisがYupになっていることを確認する
$ roslaunch mocap_optitrack mocap.launchで情報を得る
$ rostopic echo /mocap_node/turtlebot3/poseで現在の座標をわかる

〜実験開始〜
$ rosrun turtlebot3_ros ファイル名
Ctrl + Cで行動を止めたときに速度が残ってしまうので、速度を0にする方法
Tabキーを用いて、以下のすべてのコマンドを打つ
$ rostopic pub /cmd_vel geometry_msgs/Twist "linear:
x: 0.0
y: 0.0
z: 0.0
angular:
x: 0.0
y: 0.0
z: 0.0"

〜位置調整〜
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

"""