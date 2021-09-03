#!/usr/bin/env python
import rospy
import math
import numpy as np

from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt

cur_time = 0
pre_time = 0
time_now = 0
ax,ay,az = [0,0,0]
dX,dY,dZ=[0.0,0.0,0.0]
cnt = 0
start_time = 0
xx = []
yy = []
zz = []
tt = []
def callback(msg):
    global cur_time,ax,ay,az,dX,dY,dZ,pre_time,cnt,time_now, start_time
    if not start_time:
        start_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0
    c_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0
    cnt+=1
    pre_time = cur_time
    cur_time = msg.header.stamp.nsecs
    dt = (cur_time - pre_time)/1000000000.0
    pre_ax, pre_ay, pre_az = [ax,ay,az]
    ax,ay,az = [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
    if cnt > 10:
        dX+= dt*(ax+pre_ax)/2
        dY+= dt*(ay+pre_ay)/2
        dZ+= dt*(az+pre_az)/2
    #print(cnt, dX)
    xx.append(dX)
    yy.append(dY)
    zz.append(dZ)
    tt.append(c_time - start_time)
    print(c_time - start_time)
    if c_time - start_time > 1800:
        plt.plot(xx, 'r', label = 'X')
        plt.plot(yy, 'g', label = 'Y')
        plt.plot(zz, 'b', label = 'Z')
        plt.xlabel('Time (ms)')
        plt.show()
        
def imu_():
    rospy.init_node("python_imu")
    rospy.Subscriber("/os_cloud_node/imu", Imu, callback)
    rospy.spin()
if __name__=='__main__':
    imu_()
