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

IP_TS_SERVER = '192.168.1.104'
PORT_TS_SERVER = 8000
FLIGHT_MODE = ''

def callbackState(data):
    global FLIGHT_MODE
    rospy.loginfo("PX4 %s MODE", data.mode)
    FLIGHT_MODE = data.mode

class LeicaThread(Thread):
    def __init__(self, _ip, _port):
        Thread.__init__(self)
        self.mLastX = 0
        self.mLastY = 0
        self.mLastZ = 0
        self.mLastStamp = 0
        # TSReceiver Socket
        self.mSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mServer = (_ip, _port)
        self.mRun = True
        self.TSerror = False
        self.ZeroCont = 0
        # self.mLastTime = 0

    def _del_(self):
        self.mRun = False
        self.mSocket.close()

    def run(self):
        global FLIGHT_MODE
        cBufferSize = 24
        cMaxTimeOut = 1.0
	lastData = 0
	contData = 0
        self.mSocket.sendto("Hola", self.mServer)
        self.mSocket.settimeout(2) # Tocame
        # self.mLastTime = time.time()

        while (not rospy.is_shutdown()) and self.mRun:
            try:
                data = self.mSocket.recvfrom(cBufferSize)
                data = struct.unpack('fffQ', data[0])
                # print data
		if data == lastData:
			contData+=1
			print contData
		else:
			lastData = data
			contData=0
			#print contData
                # Eval if received more than 10 failed measures
                if self.ZeroCont>20 or contData>20 :
                    # Block publisher 
                    self.TSerror = True
                    rospy.logerr("Counter completed")
                    # Change mode
                    if FLIGHT_MODE == "OFFBOARD" or FLIGHT_MODE == "POSCTL":
                        self.setFlightMode("ALTCTL")
                else:
                    # Check zero in data received
                    if any(x is 0 for x in data):
                        # Update counter and let send last good measure
                        self.ZeroCont += 1
                        print self.ZeroCont # Debugging
                    else:
                        try:
                            # All ok. Update position
                            self.mLastX, self.mLastY, self.mLastZ, self.mLastStamp = data[0], data[1], data[2], data[3]
                            self.TSerror = False
                            self.ZeroCont = 0
                            # self.mLastTime = time.time()
                        except IndexError as error:
                            self.ZeroCont += 1
                            print "Captured index error while parsing input data from socket. Skipping data"
                            print error
            except socket.timeout:
                print "No data received"
                self.TSerror = True
                
                if FLIGHT_MODE == "OFFBOARD" or FLIGHT_MODE == "POSCTL":
                    self.setFlightMode("ALTCTL")

                self.mSocket.sendto("Hola", self.mServer) # Comprobar necesaria excepcion

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
    
    pub = rospy.Publisher('/uav_1/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    rospy.Subscriber("/uav_1/mavros/state", State, callbackState)

    leicaThread = LeicaThread(IP_TS_SERVER, PORT_TS_SERVER)
    leicaThread.start()

    rate = rospy.Rate(20)  # 20hz

    while not rospy.is_shutdown():
        if(not leicaThread.TSerror):
            output.header.stamp = rospy.Time.now()
            output.header.frame_id = "fcu"

            output.pose.position.x = leicaThread.mLastX
            output.pose.position.y = leicaThread.mLastY
            output.pose.position.z = leicaThread.mLastZ

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
