#Sockets
import socket
import sys
from sys import getsizeof
import cv2
import time
import struct
import threading
import message_pb2
import movement_pb2
import numpy as np

HOST = '' #An available interface
PORT = 8888 #Any port

RHOST = ''
RPORT = 8887

def send_msg(sock, msg):
    # Prefix each message with a 4-byte length (network byte order)
    msg = struct.pack('>I', len(msg)) + msg
    sock.sendall(msg)

def recv_msg(sock):
	# Read message length and unpack it into an integer
	raw_msglen = recvall(sock, 4)
	if not raw_msglen:
		return None
	msglen = struct.unpack('>I', raw_msglen)[0]
	# Read the message data
	return recvall(sock, msglen)

def recvall(sock, n):
	# Helper function to recv n bytes or return None if EOF is hit
	data = ''

	while len(data) < n:
		packet = sock.recv(n - len(data))

		if not packet:
			return None
		data += packet
	return data

def connect():
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	print 'Socket created'

	#Bind to addr/port
	try:
		s.bind((HOST, PORT))
	except socket.error as msg:
		print 'Bind failed: ' + str(msg[0]) + ' with message: ' + msg[1]
		sys.exit()

	print 'Socket bind successful'

	#Start listening on socket
	s.listen(10)
	print ' Socket is listening'

	#Wait to accept connection
	conn, addr = s.accept()

	return conn, addr	#Could include socket in return

def recv_msg_loop(conn, addr):
	while 1:
		#data = conn.recv(1024)	#Random size(1024) AND ADDED CONCAT

		#Get the data from the socket. First 4bits are the length of the packet.
		data = recv_msg(conn)

		if data is None:
			break

		#print 'Connected with ' + addr[0] + ' : ' + str(addr[1])# + '. Received data: ' + str(data)

		#Get protobuf format
		proto_img = message_pb2.Image()

		#print 'Size: ' + str(getsizeof(data))

		#Parse proto message
		proto_img.ParseFromString(data)

		#print('Width: ' + str(proto_img.rows) + ', Height: ' + str(proto_img.cols))# + ', data: ' + str(proto_img.pic))
		
		#Decode message with OpenCV
		nparr = np.fromstring(proto_img.pic, np.uint8)
		dec_img = cv2.imdecode(nparr, 1)

		#Show the image with OpenCV
		cv2.imshow("Image window", dec_img)
		cv2.waitKey(3)

		#conn.send('You are connected...')

	#conn.close() #Cannot close at this time

def send_connect(addr, port):
	s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

	print 'Addr: ' + str(addr[0]) + ", Port: " + str(port)

	#Connect to host/port
	try:
		s2.connect((addr[0], port))
	except socket.error as msg:
		print 'Connection failed: ' + str(msg[0]) + ' with message: ' + msg[1]
		sys.exit()

	print 'got here'

	return s2

def send_loop(sock, msg):
	while 1:
		send_msg(sock, msg)
	sock.close()

###
### The main of the program
###

conn, addr = connect()

t1 = threading.Thread(target=recv_msg_loop, args=(conn, addr))
t1.daemon = True
sock = send_connect(addr, 8887)
print '2'
sending = movement_pb2.Move()
sending.steering = 0.5
sending.movement = 0
msg = sending.SerializeToString()
t2 = threading.Thread(target=send_loop, args=(sock, msg))
t2.daemon = True
try:
	t1.start()
	#recv_msg_loop(conn, addr)

	print '3'
	t2.start()
	print '4'
	#send_loop(sock, "Hello")

	while 1:
		time.sleep(1)
except KeyboardInterrupt:
	sys.exit()
	print 'Process exited correctly'
