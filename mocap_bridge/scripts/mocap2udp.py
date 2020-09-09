#!/usr/bin/env python
# license removed for brevity

import rospy, tf, socket, sys, struct
from geometry_msgs.msg import PoseStamped

topic = "/mocap_client/ARBI/pose"
UDP_IP = "10.201.0.100"
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

    MESSAGE = struct.pack('ddddddd',x,y,z,qx,qy,qz,qw)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))


def talker():
    global sock

    rospy.init_node('mocap_2_udp', anonymous=True)
    sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP

    rospy.Subscriber(topic, PoseStamped, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
