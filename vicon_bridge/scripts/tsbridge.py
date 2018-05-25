#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
import socket
from geometry_msgs.msg import PoseStamped
from threading import Thread
from math import *
import signal
import sys
import time
import struct


class LeicaThread(Thread):
    cBufferSize = 24
    OFFSET=0.5

    def __init__(self, _ip, _port):
        Thread.__init__(self)
        self.mLastX = 0
        self.mLastY = 0
        self.mLastZ = 0
        self.mRefX = 0
        self.mRefY = 0
        self.mRefZ = 0
        self.mSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mServer = (_ip, _port)
        # self.mSocket.bind((_ip, _port))
        # self.mIsconnected = False
        self.mRun = True
        self.mLastTime = 0
        self.TSerror = False

    def _del_(self):
        self.mRun = False
        self.mSocket.close()

    def run(self):
        cMaxTimeOut = 1.0
        sent = self.mSocket.sendto("Hola", self.mServer)
        self.mLastTime = time.time()

        while (not rospy.is_shutdown()) and self.mRun:
            data = self.mSocket.recvfrom(LeicaThread.cBufferSize)
            if not data:
                print "No data received, disconnected from server as socket has been configured without timeout"
            else:
                # rospy.loginfo("%s %s", data, len(data[0]))
                data = struct.unpack('fffQ', data[0])
                # sys.stdout = open('TS2PX4_log.txt','awt')
                # print data
                try:
                    # Eval if any data is 0, measure delay or failed measures
                    if(any(x is 0 for x in data) or (time.time() - self.mLastTime > cMaxTimeOut)):# or deltaFilter(data)):
                        # Block publisher 
                        rospy.logerr("Recived bad measurement quality %s %s", data,time.time() - self.mLastTime)
                        self.TSerror = True
                    else:
                        self.mLastX, self.mLastY, self.mLastZ, t = data[0], data[1], data[2], data[3]
                        # rospy.loginfo("%s", data)
                        self.TSerror = False
                    # 
                    self.mLastTime = time.time()
                except IndexError as error:
                    print "Captured index error while parsing input data from socket. Skipping data"
                    print error

    # def deltaFilter(lastPose,self):
        # return abs(lastPose[1] - self.mLastX) < OFFSET or abs(lastPose[2] - self.mLastY) < OFFSET or abs(lastPose[3] - self.mLastZ) < OFFSET

def talker():
    rospy.init_node('tsbridge', anonymous=True)
    output = PoseStamped()
    # TSReceiver Socket
    pub = rospy.Publisher('/uav_1/mavros/vision_pose/pose', PoseStamped, queue_size=1)

    TCP_IP = '127.0.0.1'
    TCP_PORT = 8000
    leicaThread = LeicaThread(TCP_IP, TCP_PORT)
    leicaThread.start()

    rate = rospy.Rate(20)  # 20hz

    while not rospy.is_shutdown():
        if(not leicaThread.TSerror):
            pub.publish(output)
            output.header.stamp = rospy.Time.now()
            output.header.frame_id = "fcu"

            output.pose.position.x = leicaThread.mLastX
            output.pose.position.y = leicaThread.mLastY
            output.pose.position.z = leicaThread.mLastZ

            output.pose.orientation.x = 0
            output.pose.orientation.y = 0
            output.pose.orientation.z = 0
            output.pose.orientation.w = 1
            
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
