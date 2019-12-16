#!/usr/bin/env python
# license removed for brevity
import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
import signal
import sys
import time

FLIGHT_MODE = ''

def callbackState(data):
    global FLIGHT_MODE
    rospy.loginfo("PX4 %s MODE", data.mode)
    FLIGHT_MODE = data.mode

def setFlightMode(_flight_mode):
    global FLIGHT_MODE

    rospy.init_node('tsbridge', anonymous=True)
    rospy.Subscriber("/uav_1/mavros/state", State, callbackState)

    rospy.wait_for_service('/uav_1/mavros/set_mode')
    flight_mode_client_ = rospy.ServiceProxy('/uav_1/mavros/set_mode', SetMode)
    while not rospy.is_shutdown() and FLIGHT_MODE != _flight_mode:
        flight_mode_client_(0,_flight_mode) 
        time.sleep(0.3)

if __name__ == '__main__':
    try:
        setFlightMode("ALTCTL")
    except rospy.ROSInterruptException:
        pass
