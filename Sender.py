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
import message_pb2
import threading
import time

sendPort = 8887
sendAddress = ''
max_listen = 10

#The sender class is responsible for sending images.
#The sender represents the ROS library.
class Sender:
	def __init__(self):
		self.bridge = CvBridge()
		self.running = True
		#Setup Socket
		self.setupSockets()
		rospy.on_shutdown(self.shutdown)

	def terminate(self):
		self.running = False

	def isRunning(self):
		return self.running

	def setupSockets(self):
		#Start the receiving socket
		try:
			self.socksend = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.socksend.connect((sendAddress,sendPort))
		except socket.error as msg:
			print "Connection failed: " + str(msg)
			sys.exit()

	#This is the callback function that image subscription requires
	def imgCallback(self, msg):
		#Convert Ros Image to CV Image
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print("Error converting image: " + e)
		#Encode Image
		encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),50]
		result,encimg=cv2.imencode('.jpg',cv_image,encode_param)
		#Convert encoded image to string
		img_str = encimg.tostring()
		#Get img info
		(rows,cols,channels) = cv_image.shape
		send_str = self.makeProtoStr(rows,cols,channels,img_str)
		self.send_msg(send_str)

	def makeProtoStr (self, rows,cols,channels,img_str):
		sending = message_pb2.Image()
		sending.rows = rows
		sending.cols = cols
		sending.channels = channels
		sending.pic = img_str
		return sending.SerializeToString()

	def send_msg(self, msg):
	    # Prefix each message with a 4-byte length (network byte order)
	    msg = struct.pack('>I', len(msg)) + msg
	    try:
	    	self.socksend.send(msg)
	    except socket.error as e:
	    	print "Error sending: " + str(e) 

	def shutdown(self):
		print "Shutdown"
		self.socksend.close()

	def senderLoop(self):
		#Subscribe to the image
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.imgCallback)
		while self.running:
			time.sleep(0.001)