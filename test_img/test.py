#!/usr/bin/env python
from __future__ import print_function

import sys
import cv2
import imutils
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
	kernel=np.ones((20,20),np.uint8)
	mask=cv2.erode(mask,kernel,iterations=1)
	mask=cv2.dilate(mask,kernel,iterations=1)
        #cv2.floodFill(im_floodfill, mask, (0,0), 255)
	output = cv2.bitwise_and(image, image, mask = mask)

    	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    	cnts = imutils.grab_contours(cnts)

    	for c in cnts:
	    # compute the center of the contour
	    M = cv2.moments(c)
	    if M["m00"] != 0:
	    	cX = int(M["m10"] / M["m00"])
	    	cY = int(M["m01"] / M["m00"])
	    else:
	    	continue
	    rect=cv2.minAreaRect(c)
	    box = cv2.boxPoints(rect)
	    box = np.int0(box)
	    cv2.drawContours(output,[box],0,(0,0,255),2)
	    print(rect,cX,cY)
 
	    # draw the contour and center of the shape on the image
	    cv2.drawContours(output, [c], -1, (0, 255, 0), 2)
	    cv2.circle(output, (cX, cY), 7, (255, 255, 255), -1)
	    cv2.putText(output, "center", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
 	
	return (mask,output)


def callback():    
    cv_image = cv2.imread("/home/rashmi/AR_project/test_img/lvl1-comp.jpg")
    (rows,cols,channels) = cv_image.shape
    center_img = (cols/2, rows/2)
    #cv_image = cv_image[100:600,:] #cropped image
        	
    (mask,cv_img)=detection(cv_image,'r')
    #mask[mask>0]=1

    cv2.imshow("Original Image window", cv_image)
    cv2.imshow("Image", cv_img)

    cv2.waitKey(10000)
    

def main():
   callback()
   
   cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
