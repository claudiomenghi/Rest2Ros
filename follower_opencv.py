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

listenPort = 8887
listenAddress = ''
sendPort = 8888
sendAddress = '83.254.13.53'

class Follower:
	def __init__(self):
		self.bridge = CvBridge()
		self.sockrec = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sockrec.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		try:
			self.sockrec.bind((listenAddress, listenPort))
			self.sockrec.listen(10)
		except socket.error as msg:
			print "Connection failed"
			sys.exit()
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.connect((sendAddress,sendPort))
		self.n = 0
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

	def image_callback(self, msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		(rows,cols,channels) = cv_image.shape
		encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),50]
		result,encimg=cv2.imencode('.jpg',cv_image,encode_param)
		img_str = encimg.tostring() 

		nparr = np.fromstring(img_str, np.uint8)
		dec_img = cv2.imdecode(nparr,1)
		if self.n < 2000:
			sending = message_pb2.Image()
			sending.rows = rows
			sending.cols = cols
			sending.channels = channels
			sending.pic = img_str
			send_str = sending.SerializeToString()
			send_str = struct.pack('>I', len(send_str)) + send_str
			print getsizeof(send_str)
			self.sock.send(send_str)
			#temp = message_pb2.Image()
			#temp.ParseFromString(send_str)

			#nparr = np.fromstring(temp.pic, np.uint8)
			#dec_img = cv2.imdecode(nparr,1)
			self.n = self.n+1
		cv2.imshow("Image window", dec_img)
		cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
while 1:
	conn, addr = follower.sockrec.accept()
	data = conn.recv(1024)
	print "connected with " + addr[0] + ":" + str(addr[1]) + ", received: " + str(data)
