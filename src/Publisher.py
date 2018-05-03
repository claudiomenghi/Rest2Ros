#!/usr/bin/python
######################################################################
# This component receives missions from Rest and converts them into
# ros messages
######################################################################

import SocketServer
import socket
from sets import Set




class Publisher:
    """demonstration class only
      - coded for clarity, not efficiency
    """

    def __init__(self, sock=None):
    	self.socketset=set()
       	self.sock=None

    def run(self):
    	self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    	HOST = ''                 # Symbolic name meaning the local host
    	PORT = 50007 
        self.sock.bind((HOST, PORT))
        self.sock.listen(1)
        print('Waiting for a subscriber')
        conn, addr = self.sock.accept()
        print('subscriber added')
        self.socketset.add(conn)
        self.send("Prendi questo")

    def connect(self, host, port):
        self.sock.connect((host, port))

    def send(self, msg):

    	for socket in self.socketset:
			socket.send(msg)

    

				
def main():
	print "Running the communication manager"
	rest=Publisher()
	rest.run()
	

if  __name__ == "__main__":
	rest=Publisher()
	rest.run()
