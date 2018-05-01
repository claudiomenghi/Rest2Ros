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

import yaml


class Rest:

	def __init__(self):
		rospy.init_node('communication_manager',anonymous=False,disable_signals=True)		

	def run(self):
		while not rospy.is_shutdown():
			port = rospy.get_param('~port') 
                	topicType = rospy.get_param('~topicName')    

			server_address = ('', port)
			httpd = HTTPServer(('0.0.0.0', port),Request_Handler)
			print ("Waiting for a new mission on the port %s messages will be forwarded on the topic %s" %(port,topicType))

			httpd.serve_forever()
				
def main():
	print "Running the communication manager"
	rest=Rest()
        rest.run()
	

if  __name__ == "__main__":
	rest=Rest()
	rest.run()
