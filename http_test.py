import requests

def send_msg(addr, msg):
	req = requests.post(addr, data=msg)

	return req

def receive_msg():
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

while True:
	receive_msg()