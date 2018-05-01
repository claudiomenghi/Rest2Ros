######################################################################
# This component receives missions from Rest and converts them into
# ros messages
######################################################################

from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import SocketServer
import rospy
import socket
import struct
import cv2
import numpy as np
import sys
import unicodedata
import roslib
roslib.load_manifest("rosparam")
import rosparam

from sys import getsizeof
from std_msgs.msg import String
from cgi import parse_header, parse_multipart
from urlparse import parse_qs

class Request_Handler(BaseHTTPRequestHandler):

	def _set_header(self):
		self.send_response(200)
		self.send_header('Content-type', 'text/html')
		self.end_headers()

	def do_GET(self):
		self._set_headers()
		self.wfile.write("<html><body><h1>GET! " + self.image.getImage() + " </h1></body></html>")

	def do_HEAD(self):
		self._set_headers()

	def do_POST(self):
		content_length = int(self.headers['Content-Length']) # <--- Gets the size of data
		post_data = self.rfile.read(content_length) # <--- Gets the data itself
		postvars=parse_qs(post_data, keep_blank_values=1)
		missions=postvars['mission']
		topicName = rospy.get_param('~topicName')    

		print ('new local mission received %s' %(missions[0]))

		pub=rospy.Publisher(topicName,String, queue_size=100)
		pub.publish(missions[0])
		print ('local mission sent in ROS')

		self.send_response(200)
		self.end_headers()
		
		


