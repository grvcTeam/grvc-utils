#!/usr/bin/env python
# license removed for brevity

import rospy, tf, socket, sys, struct
from geometry_msgs.msg import PoseStamped
import time

topic = "/mocap_client/ARBI/pose"
UDP_IP = "127.0.0.1"
UDP_PORT = 21444


def callback(data):

    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    qx = data.pose.orientation.x
    qy = data.pose.orientation.y
    qz = data.pose.orientation.z
    qw = data.pose.orientation.w
    timestamp = data.header.seq

    MESSAGE = struct.pack('fffQ',x,y,z,timestamp)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))


def talker():
    global sock

    rospy.init_node('mocap_2_xr1', anonymous=True)
    sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP

    rospy.Subscriber(topic, PoseStamped, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


    # ------------ Test ------------ #
    # sock = socket.socket(socket.AF_INET, # Internet
    #                 socket.SOCK_DGRAM) # UDP
    # i = 0
    # while True:
    #     x = i
    #     y = x+1
    #     z = y+1
    #     timestamp = y+1
    #     MESSAGE = struct.pack('fffQ',x,y,z,timestamp)
    #     sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    #     print(x,y,z,timestamp)
    #     time.sleep(0.1) 
    #     i+=1
