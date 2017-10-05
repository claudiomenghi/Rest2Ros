import Receiver
#import Sender
#import Server_Receiver
import Server_Sender
import rospy
import threading
import time

print "Starting receiver..."

i = 0
image = ''

receiver = Receiver.Receiver()
t1 = threading.Thread(target=receiver.receiveServer, args=())
t1.daemon = True
t1.start()

#receiver.receiveServer()

print "Receiver started..."

print "Rospy initiated..."

sender = Server_Sender.SSender()
sock = sender.send_connect("localhost", 8888)

t2 = threading.Thread(target=sender.send_loop, args=([sock, 0.5, 0.5]))
t2.daemon = True
t2.start()

#sender = Sender.Sender()

print "Sender initiated..."

while i < 500:
	time.sleep(0.001)
	if i == 48:
		image = receiver.getImage()
	i += 1
	if i == 49:
		print str(image) + "This is the image we got from calling getImage()"