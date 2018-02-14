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
import argparse
import signal


class LeicaThread(Thread):
	cBufferSize = 1024

	def __init__(self, _ip, _port):
		Thread.__init__(self)
		self.mLastX = 0
		self.mLastY = 0
		self.mLastZ = 0
		self.mRefX = 0
		self.mRefY = 0
		self.mRefZ = 0
		self.mSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.mSocket.bind((_ip, _port))
		self.mIsconnected = False
		self.mRun = True
		self.mIp = _ip
		self.mPort = _port
		self.mState = 0;  # 0 all good; 1 disconnected; 2 listening; 3 data time out;4 unknown state

	def __del__(self):
		self.mRun = False
		self.mConn.close()
		self.mSocket.close()

	def listen(self):
		self.mState = 2
		self.mSocket.listen(1)
		try:
			print "Waiting for input connection"
			self.mConn, self.mAddr = self.mSocket.accept()
			sys.stdout.write("\033[F")  # back to previous line
			sys.stdout.write("\033[K")  # clear line
			rospy.loginfo('Connection address: %s', self.mAddr)
			self.mIsconnected = True
		except socket.error as msg:
			self.mState = 1

	def isRunning(self):
		return self.mRun

	def stop(self):
		print "Stopping connection"
		self.mRun = False
		socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect( (self.mIp, self.mPort))
		self.mSocket.close()
		print "Connection closed"

	def run(self):
		self.mLastTime = time.time()
		self.listen()

		while (not rospy.is_shutdown() and self.mRun):
			data = self.mConn.recv(LeicaThread.cBufferSize)
			self.mLastTime = time.time()
			if not data:
				print "No data received, disconnected from server as socket has been configured without timeout"
				self.mState = 1
				self.mIsconnected = False
				print "Server set as disconnected, returning to listening state"
				self.listen()
			else:
				index = data.index('}')
				parseData = data[1:index]
				parseData = parseData.split(";")
				try:
					self.mLastX, self.mLastY, self.mLastZ, t = float(parseData[0]), float(
						parseData[1]), float(parseData[2]), float(parseData[3])
				except IndexError:
					print "Captured index error while parsing input data from socket. Skipping data"


def talker(topic):
	# Init ROS
	rospy.init_node('tsbridge', anonymous=True)

	# Init local variables
	output = PoseStamped()
	pub = rospy.Publisher(topic, PoseStamped, queue_size=10)

	# Prepare connection with leica
	TCP_IP = '0.0.0.0'
	TCP_PORT = 8000
	leicaThread = LeicaThread(TCP_IP, TCP_PORT)
	leicaThread.start()
	
	def signalHandler(signum, frame):
		leicaThread.stop()
		print "Exiting"
		sys.exit()

	signal.signal(signal.SIGINT, signalHandler)

	dotCounter = 0
	while ((not leicaThread.mIsconnected) and leicaThread.isRunning()):
		print "Waiting until leica is connected"+"."*dotCounter
		dotCounter = (dotCounter+1) if (dotCounter < 5) else 0
		time.sleep(1)
		sys.stdout.write("\033[F")  # back to previous line
		sys.stdout.write("\033[K")  # clear line

	print "Connected!"
	rate = rospy.Rate(30)  # 10hz
	cMaxTimeOut = 0.5

	# Handle sigint to stop communication

	# Publishing loop
	while ((not rospy.is_shutdown()) and leicaThread.isRunning()):
		# Delete last line 
		sys.stdout.write("\033[F") #back to previous line
		sys.stdout.write("\033[K") #clear line
		delay = time.time() - leicaThread.mLastTime
		if delay > cMaxTimeOut: # check timeout
			print "WARNING! exceeded timeout of last received measure. Delay: "+str(delay)+"."*dotCounter+" "
		else:
			print "Reasonable delay. Delay: "+str(delay)+"."*dotCounter

		dotCounter = (dotCounter+1) if (dotCounter < 5) else 0

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




# --- MAIN --- #
if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Leica bridge.')
	parser.add_argument('--topic', type=str, help='Topic to publish in', required=False)
	parser.parse_args()
	try:
		if(sys.argv == 2):
			talker(parser.topic)
		else:
			talker("/uav_1/mavros/vision_pose/pose")
	except rospy.ROSInterruptException:
		pass
