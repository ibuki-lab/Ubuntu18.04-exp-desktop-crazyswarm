#!/usr/bin/env python
# coding: UTF-8
import pandas as pd
import datetime
import numpy as np
import rospy
import tf2_ros
import tf_conversions
import tf

from flock_frames_setup import Frames_setup
from flocking_ctrl import flocking_ctrl


num_cf = flocking_ctrl(1).num_cf
world_frame = Frames_setup().world_frame
child_frames = Frames_setup().children_frame
EXP_end = False
print(world_frame)
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rospy.sleep(1)
X_P_regster = [ [ ] for _ in range(num_cf)]
Y_P_regster = [ [ ] for _ in range(num_cf)]

while True:

    if EXP_end:
        break

    for id, child_frame in zip(range(num_cf), child_frames):

        cf_state = tfBuffer.lookup_transform(world_frame, child_frame, rospy.Time(0))


        X_P_regster[id].append(cf_state.transform.translation.x)
        Y_P_regster[id].append(cf_state.transform.translation.y)
    
    EXP_end = flocking_ctrl().EXP_end

data = {}
for i in range(num_cf):

    data["Xp_cf{}".format(i)] = X_P_regster[i]
    data["Yp_cf{}".format(i)] = Y_P_regster[i]

df = pd.DataFrame(data)
df.to_csv("flocking_ctl_{}_{}".format(datetime.date.today()))



        