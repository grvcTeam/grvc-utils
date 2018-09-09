#!/usr/bin/env python
# license removed for brevity
import rospy, tf, time
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State
from threading import Timer

topicPlatform = "vicon_client/X6AEROARMS/pose"
topicLocal = "uav_1/mavros/local_position/pose"
topicState = "uav_1/mavros/state"

ON,OFF = "\033[01;32mON\033[00m","\033[01;31mOFF\033[00m"

vicon_stamp = []
local_pose = Point()
mavstate = ""

def is_estimator_out():
    return (local_pose.x > -0.1 and local_pose.x < 0.1) and (local_pose.y > -0.1 and local_pose.y < 0.1) and (local_pose.z > -0.1 and local_pose.z < 0.1)

def callback_vicon(data):
    global vicon_stamp
    vicon_stamp = [data.header.stamp.secs, data.header.stamp.secs]

def callback_local(data):
    global local_pose
    local_pose = data.pose.position

def callback_state(data):
    global mavstate
    mavstate = "\033[01;33m" + data.mode + "\033[00m"

def talker():
    global vicon_stamp
    rospy.init_node('monitor', anonymous=True)
    last_stamp = []
    vicon_state = OFF
    estimator_state = OFF

    rospy.Subscriber(topicPlatform, PoseStamped, callback_vicon)
    rospy.Subscriber(topicLocal, PoseStamped, callback_local)
    rospy.Subscriber(topicState, State, callback_state)

    rate = rospy.Rate(50) # 50Hz
    
    while not rospy.is_shutdown():
        if is_estimator_out(): 
            estimator_state = OFF
        else: 
            estimator_state = ON

        if last_stamp == vicon_stamp:
                vicon_state = OFF
        else:
                vicon_state = ON

        print "[{0}] Estimator - [{1}] | Vicon - [{2}] \r".format(mavstate,estimator_state, vicon_state),
        vicon_stamp = last_stamp
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass