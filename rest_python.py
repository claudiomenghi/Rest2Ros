#!/usr/bin/env python
"""
Very simple HTTP server in python.
Usage::
    ./dummy-web-server.py [<port>]
Send a GET request::
    curl http://localhost
Send a HEAD request::
    curl -I http://localhost
Send a POST request::
    curl -d "foo=bar&bin=baz" http://localhost
"""
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import SocketServer
import rospy
import socket
import struct
import cv2
import numpy as np
import sys
import unicodedata
from sys import getsizeof
from std_msgs.msg import String

class Handler(BaseHTTPRequestHandler):
	#def __init__(self, request, client_address, socketServer):
	#	print 'request received'
        	
	#	pass
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
		print 'post request received %s' %(str(post_data))
        	pub=rospy.Publisher('task_array',String, queue_size=10)
		pub.publish((str(post_data)))
		print 'message sent in ROS'
        	#self._set_headers()
        	#self.wfile.write("<html><body><h1>POST! " + str(post_data) + " </h1></body></html>")
		#if data != None:
		#	print "Publishing data %s" %(data)
		#	self.cmd_vel.publish(data)


class Rest:

	def __init__(self):
		rospy.init_node('Receiver', anonymous=False)
		# What function to call when you ctrl + c    
		self.cmd_vel = rospy.Publisher('task_array', String, queue_size=100)
	def run(self):
		port=8886
		server_address = ('', port)
		httpd = HTTPServer(('127.0.0.1', port),Handler)
		print 'Starting httpd...'
		try:
			httpd.serve_forever()
		except KeyboardInterrupt:
			pass



	

if __name__ == "__main__":
	from sys import argv
	rest=Rest()
	if len(argv) == 2:
		rest.run()
	else:
		rest.run()
