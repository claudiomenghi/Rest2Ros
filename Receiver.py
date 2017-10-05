import rospy
import socket
import struct
import cv2
import numpy as np
import sys
import unicodedata
from sys import getsizeof
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import movement_pb2
import threading
import time
from geometry_msgs.msg import Twist
#

listenPort = 8888
listenAddress = ''
max_listen = 10

#The Receiver class is responsible for receiving speed and angle values from the 
#SSender and publishing them to ROS.
#The Receiver represents the ROS library.
class Receiver:
	def __init__(self):
		self.setupSockets()
		rospy.init_node('Receiver', anonymous=False)
		# What function to call when you ctrl + c    
		rospy.on_shutdown(self.shutdown)
		self.cmd_vel = rospy.Publisher('task_array', String, queue_size=10)

	def setupSockets(self):
		#Start the receiving socket
		self.sockrec = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sockrec.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		try:
			self.sockrec.bind((listenAddress, listenPort))
			self.sockrec.listen(max_listen)
		except socket.error as msg:
			print "Connection failed"
			sys.exit()

	def receiveServer(self):
		conn, addr = self.sockrec.accept()
		
		while 1:
			#Get the data from the socket. First 4bits are the length of the packet.
			data = self.recv_msg(conn)

			if data is None:
				break

			print "Making cmd"

			if data != None:
				print "Publishing data %s" %(data)
				self.cmd_vel.publish(data)


	def recv_msg(self, sock):
		# Read message length and unpack it into an integer
		raw_msglen = self.recvall(sock, 4)
		if not raw_msglen:
			return None
		msglen = struct.unpack('>I', raw_msglen)[0]
		# Read the message data
		return self.recvall(sock, msglen)

	def recvall(self, sock, n):
		# Helper function to recv n bytes or return None if EOF is hit
		data = ''

		while len(data) < n:
			packet = sock.recv(n - len(data))

			if not packet:
				return None
			data += packet
		return data

	def shutdown(self):
		print "Shutdown"
		self.sockrec.close()
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
		self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
		rospy.sleep(1)
