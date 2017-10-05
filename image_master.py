import Sender
import Server_Receiver
import threading
import time

class ImageReceiver:
	def __init__(self):
		self.receiver = Server_Receiver.SReceiver()
		t1 = threading.Thread(target=self.receiver.recv_msg_loop, args=([]))
		t1.daemon = True	#Only needed for attached thread
		t1.start()
		self.sender = Sender.Sender()
		t2 = threading.Thread(target=self.sender.senderLoop, args=([]))
		t2.daemon = True	#Only needed for attached thread
		t2.start()
		time.sleep(1)
	def getImage(self):
		return self.receiver.getImage()
	def terminate(self):
		self.sender.terminate()