#!/usr/bin/env python
# license removed for brevity
import rospy, tf, socket
from geometry_msgs.msg import PoseStamped
from threading import Thread
from math import *
import signal
import sys
import time

class LeicaThread(Thread):
	cBufferSize = 1024

	def __init__(self, _ip, _port):
		Thread.__init__(self)
		self.mLastX = 0;
		self.mLastY = 0;
		self.mLastZ = 0;
		self.mRefX = 0;
		self.mRefY = 0;
		self.mRefZ = 0;
		self.mSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.mSocket.bind((_ip, _port))
		self.mIsconnected = False
		self.mRun = True
		self.mState = 0;	# 0 all good; 1 disconnected; 2 listening; 3 data time out;4 unknown state

	def _del_(self):
		self.mRun = False
		self.mConn.close()
		self.mSocket.close()

	def listen(self):
		self.mState = 2
		self.mSocket.listen(1)
		try:
			self.mConn, self.mAddr = self.mSocket.accept()
	    		rospy.loginfo('Connection address: %s', self.mAddr)
			self.mIsconnected = True;
		except socket.error as msg:
			self.mState = 1


	def run(self):
		self.mLastTime = time.time()
		self.listen()

		while (not rospy.is_shutdown()) and self.mRun:
			data = self.mConn.recv(LeicaThread.cBufferSize)
			self.mLastTime = time.time()
			if not data:
				print "No data received, disconnected from server as socket has been configured without timeout"
				self.mState = 1
				self.mIsconnected = False;
				print "Server set as disconnected, returning to listening state"
				self.listen()
			else:
				#sys.stdout = open('TS2PX4_log.txt','awt')
				print data
				if len(data) >= 37 and "{" in data:
					index_init = data.index('{')
					data = data[index_init:]
					if "}" in data:
						index_end = data.index('}')
						parseData = data[1:index_end]
						parseData = parseData.split(";")
						try:
							print parseData
							self.mLastX, self.mLastY, self.mLastZ, t = float(parseData[0]), float(parseData[1]), float(parseData[2]), float(parseData[3])
							print "--------"
						except IndexError:
							print "Captured index error while parsing input data from socket. Skipping data"

def talker():
    rospy.init_node('tsbridge', anonymous=True)

    output = PoseStamped()

    # TSReceiver Socket

    pub = rospy.Publisher('/uav_1/mavros/vision_pose/pose', PoseStamped, queue_size=10)

    TCP_IP = '0.0.0.0'
    TCP_PORT = 8000
    leicaThread = LeicaThread(TCP_IP, TCP_PORT)
    leicaThread.start();

    while not leicaThread.mIsconnected:
	print "Waiting until leica is connected"
	time.sleep(1)

    rate = rospy.Rate(20) # 20hz
    cMaxTimeOut = 2.0;
    while not rospy.is_shutdown():

	#if time.time() - leicaThread.mLastTime > cMaxTimeOut: # check timeout
		#print "WARNING! exceeded timeout of last received measure"

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
