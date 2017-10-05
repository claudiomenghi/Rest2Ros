import image_master
import movement_master
import time
import numpy as np
import cv2

rec = image_master.ImageReceiver()
#send = movement_master.Steering()

time.sleep(2)

#print "This is the image"
img = rec.getImage()

time.sleep(1)

print img

print "THIS IS THE IMAGE"

#Decode message with OpenCV
#nparr = np.fromstring(img, np.uint8)
#dec_img = cv2.imdecode(nparr, 1)

#Show the image with OpenCV
#cv2.imshow("Image window", dec_img)
#cv2.waitKey(3)

#time.sleep(2)

#cv2.destroyAllWindows()

#print "Setting commands"
#send.setAngle(1.6)
#time.sleep(1)
#send.setSpeed(2)

#time.sleep(1)

#send.terminate()
rec.terminate()

print "finished"