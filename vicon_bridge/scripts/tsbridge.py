#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
import socket
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from threading import Thread
from math import *
import signal
import sys
import time
import struct

FLIGHT_MODE = ''

def callbackState(data):
    global FLIGHT_MODE
    rospy.loginfo("PX4 %s MODE", data.mode)
    FLIGHT_MODE = data.mode

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
        global FLIGHT_MODE
        cMaxTimeOut = 1.0
        self.mSocket.sendto("Hola", self.mServer)
        self.mLastTime = time.time()

        while (not rospy.is_shutdown()) and self.mRun:
            self.mSocket.settimeout(0.5)
            try:
                data = self.mSocket.recvfrom(LeicaThread.cBufferSize)
                data = struct.unpack('fffQ', data[0])
                self.TSerror = False
                # print data
                try:
                    # Eval if any data is 0
                    if any(x is 0 for x in data):# or deltaFilter(data)):
                        # Block publisher 
                        rospy.logerr("Recived bad measurement quality -> input data: %s", data)
                        if FLIGHT_MODE == "OFFBOARD":
                            self.TSerror = True
                            self.setFlightMode("ALTCTL")
                    else:
                        self.TSerror = False
                        self.mLastX, self.mLastY, self.mLastZ, t = data[0], data[1], data[2], data[3]
                        self.mLastTime = time.time()
                except IndexError as error:
                    print "Captured index error while parsing input data from socket. Skipping data"
                    print error
            except socket.timeout:
                print "No data received"
                self.mSocket.sendto("Hola", self.mServer)
                if FLIGHT_MODE == "OFFBOARD":
                    self.TSerror = True
                    self.setFlightMode("ALTCTL")

    def setFlightMode(self, _flight_mode):
        global FLIGHT_MODE
        rospy.wait_for_service('/uav_1/mavros/set_mode')
        flight_mode_client_ = rospy.ServiceProxy('/uav_1/mavros/set_mode', SetMode)
        while not rospy.is_shutdown() and FLIGHT_MODE != _flight_mode:
            flight_mode_client_(0,_flight_mode) 
            time.sleep(0.3)

def talker():
    rospy.init_node('tsbridge', anonymous=True)
    output = PoseStamped()
    # TSReceiver Socket
    pub = rospy.Publisher('/uav_1/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    rospy.Subscriber("/uav_1/mavros/state", State, callbackState)

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
