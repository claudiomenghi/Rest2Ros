#!/usr/bin/python
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
import rosparam
from Request_Handler import Request_Handler
from sys import getsizeof
from std_msgs.msg import String
from cgi import parse_header, parse_multipart
from urlparse import parse_qs




class Rest:

	def __init__(self):
		rospy.init_node('communication_manager', anonymous=False,disable_signals=True)
		# What function to call when you ctrl + c    
		self.cmd_vel = rospy.Publisher('local_mission', String, queue_size=100, latch=True)
		self.cmd_vel.publish("rest component activated")

	def run(self):
		while not rospy.is_shutdown():
				#roslib.load_manifest("rosparam")
				#port = rospy.get_param('port')   
				port=13000
                		
				server_address = ('', port)
				httpd = HTTPServer(('0.0.0.0', port),Request_Handler)
				print ('Waiting for a new mission on the port ', port)

				
				httpd.serve_forever()
				
	
	

if __name__ == "__main__":
	from sys import argv
	rest=Rest()
	if len(argv) == 2:
		rest.run()
	else:
		rest.run()
