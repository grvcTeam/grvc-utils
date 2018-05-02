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
    cBufferSize = 44

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

    def _del_(self):
        self.mRun = False
        self.mSocket.close()

    def run(self):
        sent = self.mSocket.sendto("Hola", self.mServer)
        self.mLastTime = time.time()

        while (not rospy.is_shutdown()) and self.mRun:
            data = self.mSocket.recvfrom(LeicaThread.cBufferSize)
            self.mLastTime = time.time()
            if not data:
                print "No data received, disconnected from server as socket has been configured without timeout"
            else:
                data = struct.unpack('L4f', data[0])
                # sys.stdout = open('TS2PX4_log.txt','awt')
                # print data
                try:
                    # print parseData
                    self.mLastX, self.mLastY, self.mLastZ, t = data[1], data[2], data[3], data[0]
                    # print "--------"
                except IndexError:
                    print "Captured index error while parsing input data from socket. Skipping data"


def talker():
    rospy.init_node('tsbridge', anonymous=True)
    watchdogCounter = 0
    delayFilter = 5
    output = PoseStamped()
    # TSReceiver Socket
    pub = rospy.Publisher('/uav_1/mavros/vision_pose/pose', PoseStamped, queue_size=1)

    TCP_IP = '127.0.0.1'
    TCP_PORT = 8000
    leicaThread = LeicaThread(TCP_IP, TCP_PORT)
    leicaThread.start()

    rate = rospy.Rate(20)  # 20hz
    cMaxTimeOut = 1.0

    while not rospy.is_shutdown():
		#Y=X=Z with lost prism TS send 0,0,0 measure
        if (time.time() - leicaThread.mLastTime > cMaxTimeOut) or (leicaThread.mLastY == leicaThread.mLastX and leicaThread.mLastX == leicaThread.mLastZ):
            if watchdogCounter < 5:
                watchdogCounter = watchdogCounter + 1
                output.header.stamp = rospy.Time.now()
        else:
            output.header.stamp = rospy.Time.now()
            watchdogCounter = 0

        if delayFilter == 0:
            if (abs(output.pose.position.x - leicaThread.mLastY) < 0.4 or abs(output.pose.position.y - leicaThread.mLastX) < 0.4 or abs(output.pose.position.z - leicaThread.mLastZ) < 0.4):
                # NED Conversion
                output.pose.position.x = leicaThread.mLastY
                output.pose.position.y = leicaThread.mLastX
                output.pose.position.z = -leicaThread.mLastZ
        else:
            delayFilter = delayFilter - 1
            # NED Conversion
            output.pose.position.x = leicaThread.mLastY
            output.pose.position.y = leicaThread.mLastX
            output.pose.position.z = -leicaThread.mLastZ

        # output.header.stamp = rospy.Time.now()
        output.header.frame_id = "fcu"
        output.pose.orientation.x = 0
        output.pose.orientation.y = 0
        output.pose.orientation.z = 0
        output.pose.orientation.w = 1

        pub.publish(output)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
