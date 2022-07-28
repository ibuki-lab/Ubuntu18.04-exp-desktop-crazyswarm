#!/user/bin/env python
# coding: UTF-8

#################### crazyflie/optitrackの事前設定 ####################

########## (1) 制御方法/デッキの有無の設定 ##########
# ~/crazyswarm/ros_ws/src/crazyswarm/launch/hover_swarm.launchで以下の設定を行う
# 22: controller: 所望の番号(通常は1: PIDを選択する)
# 36: object_tracking_type: "libobjecttracker(デッキあり(cf1~cf13))" / "motionCapture(デッキなし(cf14))"

########## (2) crazyflieの初期位置の設定 ##########
# ~/crazyswarm/ros_ws/src/crazyswarm/launch/allCrazyflies.yamlで初期位置の座標を設定する
# 同じ初期位置のcrazyflieが存在するとエラーを吐くので注意する

########## (3) optitrack側の設定 ##########
# View/Asset Paneで所望の剛体を登録する
# Streaming IDは~/catkin_ws/src/mocap_optitrack/config/mocap.yamlのIDと一致させる

########## (4) Data Streaming Paneの設定(2022/1/19時点) ##########
# View/Data Streaming Paneで以下の設定を行う
# Broadcast Frame: On
# Local Interface: 192.168.50.6
# Labeled Markers: On
# Unlabeled Markers: On(剛体登録していないマーカーも読み取ってくれる)
# Asset Markers: Off
# Rigid Bodies: On
# Up Axis: Zup(Yupはturtlebot3なら可能である)
# Remote Trigger: Off
# Transmission Type: Multicast
# Subject Prefix: Off
# Visual3D Compatible: Off
# Scale: 1
# Command Port: 1510
# Data Port: 1511
# Multicast Interface: 239.255.42.99
# Multicast as Broadcast: On
# Socket Size: 1000000
# Broadcast VRPN Data: On
# VRPN Broadcast Port: 3883
# Broadcast Tracked Data: On





#################### crazyflieの飛ばし方 ####################

########## (1) 使用するcrazyflieの番号を選択する ##########
# 設定した初期位置の座標に置く(剛体登録しているcf14であれば、どこでも大丈夫である)
# 実際のcrazyflieをフィールドに置くときには、crazyflieの矢印の方向をxが正の向きと一致させるように置くこと
# chooser.pyのタブは閉じてもよい
# batteryの欄から、こまめに電池残量を確認すること
""" $ cd ~/crazyswarm/ros_ws/src/crazyswarm/scripts/ """
""" $ python chooser.py """

########## (2) optitrackから情報を得る ##########
# optitrackから情報を得る
""" $ roslaunch mocap_optitrack mocap.launch """
# Rvizで位置関係を確認する
# Rvizのタブは閉じないこと
""" $ roslaunch crazyswarm hover_swarm.launch """

########## (3) 所望のプログラムを動かす ##########
# 実行したい箇所まで移動する
""" $ cd ~/crazyswarm/ros_ws/src/crazyswarm/turtlebot3/ """
# PATHを通す
""" $ export PYTHONPATH=$PYTHONPATH:~/crazyswarm/ros_ws/src/crazyswarm/scripts """
# 所望のプログラムを実行する
# ファイル/~/所望のプログラム/プロパティ/アクセス権でプログラムとして実行可能にチェックを入れる
# chmodコマンドでも権限を与えられる
# rosrunでは動かないので注意する
""" $ python sample.py """

########## (4) エラーの対処法 ########## 
# 全てのコマンドをキーボードから打つのは大変なので、tabキーを活用すること
""" $ rosservice call /land "groupMask:0 """
"""   height: 0.02                       """ # height(高さ)は0でも大丈夫である
"""   duration:                          """ # durationa(間隔)
"""   secs: 5                            """ # secs(秒)は0秒にするとほぼ落下と同じになってしまう
"""   nsecs:0"                           """ # nsecs(10^-6秒)

########## (5) エラー後のプログラムの実行方法 ##########
# 以下のコマンドを一度「Ctrl + C」でキャンセルすること
""" $ roslaunch mocap_optitrack mocap.launch """
""" $ roslaunch crazyswarm hover_swarm.launch """
# chooser.pyでreboot(再起動)を行う必要がある
""" $ python chooser.py """




#################### Turtlebot3の使い方 ####################

########## (1) ROS masterをたてる ##########
""" $ roscore """

########## (2) Turtlebot3側の準備 ##########
# ssh ユーザ名@IPアドレス(IPアドレスはconfigで調べられる)
# Turtlebot3を起動した直後だと、すぐにはsshで繋がらないので注意すること
""" $ ssh ubuntu@192.168.0.166(burger) """
""" $ PW: turtlebot                    """
""" $ ssh ubuntu@192.168.0.230(waffle) """
# 通信を受け取る状態に以下のコマンドで入る
""" $ roslaunch turtlebot3_bringup turtlebot3_robot.launch """
# bashrcで通信方法の設定を見直す(「Ctrl + S」で上書き保存/「Ctrl + X」で終了)
""" $ sudo nano ~/.bashrc """
# export ROS_MASTER_URI=http://192.168.0.225:11311 (デスクトップの場合)
# export ROS_MASTER_URI=http://192.168.0.152:11311 (ノートパソコンの場合)
# export ROS_IP=192.168.50.7
# export TURTLEBOT3_MODEL=burger
# export TURTLEBOT3_MODEL=waffle

########## (3) optitrackから情報を得る ##########
# Reset Pivotをしない場合は、フィールドのどこかに置く
# もし必要であれば、正面がxが正の方向になるように置き、optitrackの画面で剛体を左クリックで囲み、右クリックでReset Pivotをする
# optitrackから情報を得る
""" $ roslaunch mocap_optitrack mocap.launch """
# 座標位置の情報が送られてきているかどうかを確認する
""" $ rostopic echo /mocap_node/turtlebot3/pose """

########## (4) 所望のプログラムを動かす ##########
# 以下のコマンドでキーボード入力で操作できる
""" $ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch """
# 所望のプログラムを実行する
""" $ rosrun turtlebot3_ros sample.py """

########## (5) 速度のログが残ってしまっているときの対処法 ##########
# 全てのコマンドをキーボードから打つのは大変なので、tabキーを活用すること
""" $ rostopic pub /cmd_vel geometry_msgs/Twist "linear: """
"""   x: 0.0                                             """
"""   y: 0.0                                             """
"""   z: 0.0                                             """
"""   angular:                                           """
"""   x: 0.0                                             """
"""   y: 0.0                                             """
"""   z: 0.0"                                            """
