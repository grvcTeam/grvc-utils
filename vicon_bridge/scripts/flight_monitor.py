#!/usr/bin/env python
# license removed for brevity
import rospy, tf, time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

posePlatform = ""
poseTool = ""
local_pose = ""
mavstate = ""

def callback_Platform(data):
    global posePlatform
    posePlatform = data

def callback_Tool(data):
    global poseTool
    poseTool = data

def callback_local(data):
    global local_pose
    local_pose = data

def callback_state(data):
    global mavstate
    mavstate = data.mode


def talker():
    rospy.init_node('monitor', anonymous=True)

    rospy.Subscriber("vicon_client/X6AEROARMS/pose", PoseStamped, callback_Platform)
    # rospy.Subscriber("vicon_client/CrawlerAeroarmsUSE/pose", PoseStamped, callback_Tool)
    rospy.Subscriber("uav_1/mavros/local_position/pose", PoseStamped, callback_local)
    rospy.Subscriber("uav_1/mavros/state", State, callback_state)

    x,x_e,y,y_e,z,z_e = 0,0,0,0,0,0

    rate = rospy.Rate(50) # 1Hz
    time.sleep(1)

    while not rospy.is_shutdown():
        x,y,z = abs(posePlatform.pose.position.x - local_pose.pose.position.x), abs(posePlatform.pose.position.y - local_pose.pose.position.y), abs(posePlatform.pose.position.z - local_pose.pose.position.z)
        if x > x_e: x_e = x
        if y > y_e: y_e = y
        if z > z_e: z_e = z
        print "[{0}] Diff Vicon-Platform x:{1:4f} y:{2:4f} z:{3:4f}\r".format(mavstate,x,y,z),
        # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
