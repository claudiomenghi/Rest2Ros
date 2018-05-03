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
import random
import sys
import time
import threading

from Request_Handler import Request_Handler
from Publisher import Publisher
from sys import getsizeof
from std_msgs.msg import String
from cgi import parse_header, parse_multipart
from urlparse import parse_qs

from ms2_kth.msg import * 

import yaml


class Rest:

	def __init__(self):
		rospy.init_node('communication_manager',anonymous=False,disable_signals=True)	
		self.publisher=Publisher()	
		thread = threading.Thread(target=self.publisher.run, args=())
        	thread.daemon = True                            # Daemonize thread
        	thread.start()

	def publishLocations(self,msg=ms2_kth.msg.MissionLocations()):
			locations = msg.data
			print ("Sending to the subscribers the set of locations %s" %str(locations))
			self.publisher.send("locations %s" % str(locations))
	
	def publishActions(self,msg=ms2_kth.msg.MissionActions()):
			action = msg.data
			print ("Sending to the subscribers the set of actions %s" %str(actions))
			self.publisher.send("actions %s" %str(actions))

	def run(self):

		rospy.Subscriber(rospy.get_param('~mission_locations'), ms2_kth.msg.MissionLocations, self.publishLocations) 	
		rospy.Subscriber(rospy.get_param('~mission_actions'), ms2_kth.msg.MissionActions, self.publishActions) 	# 

		

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
