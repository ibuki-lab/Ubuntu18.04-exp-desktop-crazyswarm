#!/usr/bin/env python 
# coding:UTF-8

# #!/usr/bin/env python：pythonを使用するという宣言
# coding:UTF-8：日本語表記に必要

from numpy.lib.shape_base import apply_along_axis
import time
import rospy
from math import cos, sin
import numpy as np
import matplotlib.pyplot as plt
# import tf2_ros
# import tf_conversions
import tf
from geometry_msgs.msg import Twist, PoseStamped

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
# BURGER_ip = 192.168.0.166

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82
# WAFFLE_ip = 192.168.0.230

L = 0.1 # [m]
kp = 0.2
# kp = 0.1は一番惜しい動きをしている
ki = 0.01 # 0と0.01は変化なし
kd = 0
twist = Twist()
# rate2 = 1/100

def callback(data):
    global twist, ki, kp, pre_V, pre_W, aft_V, aft_W
    # print("Experiment Start!")
    # while True:
        # try:
            # t = self.tfBuffer.lookup_transform(self.world_frame, self.turtle_frame, rospy.Time(0))
        # except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # rospy.sleep(0.5)
            # continue
    
        # クオータニオン(四元数)の取得
            # self.Quaternion = (t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
        # オイラー角の取得
            # self.RPY = tf_conversions.transformations.euler_from_quaternion(self.Quaternion)

        # self.Xp = t.transform.translation.x
        # self.Yp = t.transform.translation.y
        # Xd, Yd：所望の座標
        # Xp, Yp：現在の座標

    t = tf.transformations.euler_from_quaternion((data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))

    Xp = data.pose.position.x
    Yp = data.pose.position.y
    theta = t[2]  
    # 3行1列の行列を定義する
    vec = np.array([[0], [0], [0]])
    # [x], [y], [z]と座標を示す
    Xc = Xp + L * cos(theta)
    Yc = Yp + L * sin(theta)

    Vtil = (vec[0, 0] - Xc)    
    # 1行1列目の要素、すなわち所望の座標[x]から、Xpを引いている
    Wtil = (vec[1, 0] - Yc) 
    # 2行1列目の要素。すなわち所望の座標[y]から、Ypを引いている
    # V, W：turtlebot3側に入力するパラメータ
    # self.V = Vtil*math.cos(self.RPY[2]) + Wtil*math.sin(self.RPY[2])
    V = Vtil * kp * cos(theta) + Wtil * kp * sin(theta)
    # self.W = (-Vtil*math.sin(self.RPY[2]) + Wtil*math.cos(self.RPY[2]))*(1/self.L)
    W = -Vtil * kp * sin(theta) + Wtil * kp * cos(theta) * (1/L)
    
    pre_V.append(V)
    pre_W.append(W)
    # VとWには制限があるので、ifで最大値を超えるようであれば制限するようにする

    if V > BURGER_MAX_LIN_VEL:
        V = BURGER_MAX_LIN_VEL
    elif V < -BURGER_MAX_LIN_VEL:
        V = -BURGER_MAX_LIN_VEL
    else:
        V = V
    
    if W > BURGER_MAX_ANG_VEL:
        W = BURGER_MAX_ANG_VEL
    elif W < -BURGER_MAX_ANG_VEL:
        W = -BURGER_MAX_ANG_VEL
    else:
        W = W

    aft_V.append(V)
    aft_W.append(W)

    """
    if V > BURGER_MAX_LIN_VEL and W > BURGER_MAX_ANG_VEL:
        V = BURGER_MAX_LIN_VEL
        W = BURGER_MAX_ANG_VEL
    elif V < -BURGER_MAX_LIN_VEL and W > BURGER_MAX_ANG_VEL:
        V = -BURGER_MAX_LIN_VEL
        W = BURGER_MAX_ANG_VEL
    elif V > BURGER_MAX_LIN_VEL and W < BURGER_MAX_ANG_VEL:
        V = BURGER_MAX_LIN_VEL
        W = -BURGER_MAX_ANG_VEL 
    elif V < BURGER_MAX_LIN_VEL and W < -BURGER_MAX_ANG_VEL:
        V = -BURGER_MAX_LIN_VEL
        W = -BURGER_MAX_ANG_VEL
    elif V > BURGER_MAX_LIN_VEL:
        V = BURGER_MAX_LIN_VEL
    elif V < BURGER_MAX_LIN_VEL:
        V = -BURGER_MAX_LIN_VEL
    elif W > BURGER_MAX_ANG_VEL:
        W = BURGER_MAX_ANG_VEL
    elif W < BURGER_MAX_ANG_VEL: 
        W = -BURGER_MAX_ANG_VEL

    """

    # print("after changing")
    # print(V, W)

    """
    if V > WAFFLE_MAX_LIN_VEL and W > WAFFLE_MAX_ANG_VEL:
        V = WAFFLE_MAX_LIN_VEL
        W = WAFFLE_MAX_ANG_VEL
    elif V < -WAFFLE_MAX_LIN_VEL and W > WAFFLE_MAX_ANG_VEL:
        V = -WAFFLE_MAX_LIN_VEL
        W = WAFFLE_MAX_ANG_VEL
    elif V > WAFFLE_MAX_LIN_VEL and W < WAFFLE_MAX_ANG_VEL:
        V = WAFFLE_MAX_LIN_VEL
        W = -WAFFLE_MAX_ANG_VEL
    elif V < WAFFLE_MAX_LIN_VEL and W < -WAFFLE_MAX_ANG_VEL:
        V = -WAFFLE_MAX_LIN_VEL
        W = -WAFFLE_MAX_ANG_VEL
    elif V > WAFFLE_MAX_LIN_VEL:
        V = WAFFLE_MAX_LIN_VEL
    elif V < WAFFLE_MAX_LIN_VEL:
        V = -WAFFLE_MAX_LIN_VEL
    elif W > WAFFLE_MAX_ANG_VEL:
        W = WAFFLE_MAX_ANG_VEL
    elif W < WAFFLE_MAX_ANG_VEL:
        W = -WAFFLE_MAX_ANG_VEL
    """
            
    # 所望の座標に近くなったら、処理を中止する
    # if self.Xd - self.Xp < 0.05 and self.Yd - self.Yp < 0.05:
        # self.V = 0.0
        # self.W = 0.0
        # rospy.sleep(5)

    # BURGER_MAX_LIN_VEL = 0.22
    # BURGER_MAX_ANG_VEL = 2.84 

    twist.linear.x = V
    # twist.linear.y = 0.0
    # twist.linear.z = 0.0
    # twist.angular.x = 0.0
    # twist.angular.y = 0.0
    twist.angular.z = W

    print("after adjustment")
    print(V, W)

    # while not rospy.is_shutdown():
        # self.pub.publish(twist)
        # print("During the Experiment") 

    # break
        
    # print("Experiment finish!")

def controller_input():
    global twist, pre_V, pre_W, aft_V, aft_W
    # ワールド座標の定義
    # world_frame = "world"
    # エージェント座標(turtlebot3)の定義
    # turtle_frame = "turtlebot3" 

    rospy.init_node("position_control", anonymous=True)
    rate = rospy.Rate(100) 
    # 1秒あたりに100回ループが回ると推定できる
    # 処理時間が1/100秒を超えていない限りは可能である
    
    # rospyにノード名を通知する
    # rospyがこの情報を得ない限り、ROSのMasterと通信を始めることができない 
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    # pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    # pub = rospy.Publisher("chatter", String, queue_size=10) 
    # ノードがchatterというトピックにStringというメッセージのタイプで送っているという宣言
    rospy.Subscriber("/mocap_node/turtlebot3/pose", PoseStamped, callback)

    pre_V = []
    pre_W = []
    aft_V = []
    aft_W = []
    start = time.time()
    
    # tfフレームの初期化
    # self.tfBuffer = tf2_ros.Buffer()

    while not rospy.is_shutdown():
        pub.publish(twist)
        print("During the Experiment")
        rate.sleep()
        if time.time() - start > 50:
            break
    
    plt.plot(pre_V, label="pre_V")
    plt.plot(pre_W, label="pre_omega")
    plt.plot(aft_V, label="aft_V")
    plt.plot(aft_W, label="aft_omega")
    plt.show


if __name__ == '__main__':
    # 所望の座標を入力する
    # Xd = float(input("Xd:"))
    # Yd = float(input("Yd:"))

    controller_input()

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