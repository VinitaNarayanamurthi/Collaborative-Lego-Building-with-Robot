#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def detection(image,color):
	# define the list of boundaries
	boundaries = [
	([0, 70, 50], [10, 255, 255]),
	([0, 0, 100], [100, 100, 255]),
	([25, 146, 190], [62, 174, 255]),
	([103, 86, 65], [145, 133, 128])]
	
    	hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	
	color_def={'r':0,'g':1,'b':2,'y':3}
	# select the color boundaries
	color_sel=color_def.get(color,0)
	(lower, upper)=boundaries[color_sel]
	# create NumPy arrays from the boundaries
	lower = np.array(lower, dtype = "uint8")
	upper = np.array(upper, dtype = "uint8")

	# find the colors within the specified boundaries and apply
	# the mask
	mask = cv2.inRange(hsv, lower, upper)
	kernel=np.ones((5,5),np.uint8)
	mask=cv2.erode(mask,kernel,iterations=1)
	output = cv2.bitwise_and(image, image, mask = mask)
 	return output
	
class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
    cv_img=detection(cv_image,'r')
    cv2.imshow("Image window", cv_img)
    cv2.imshow("Original Image window", cv_image)
    cv2.waitKey(3)


def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
