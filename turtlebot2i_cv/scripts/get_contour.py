#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge=CvBridge()
x_min, x_max = 160, 480
y_min, y_max = 0, 480

def get_distance(img):
    cv_image = bridge.imgmsg_to_cv2(img, "passthrough")
    cv_image = cv2.blur(cv_image, (5,5))
    cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
    a = np.array(cv_image[y_min:y_max, x_min:x_max])
    b = np.sum(a)/((x_max - x_min)*(y_max - y_min)) #average of array
    c = np.where(200 > cv_image, 2000, cv_image)
    d = np.where(c > 1500, 2000, c)
    thresh = np.where(d == 2000, 0, 255)
    thresh = np.uint8(thresh)

    cv2.imshow("thresh", thresh)
    # thresh = cv2.adaptiveThreshold(cv_image,np.max(cv_image),cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
    _, cnts, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)
    # draw contours on the original image
    image_copy = np.zeros((480, 640, 3), dtype=np.uint8)
    # cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    for c in cnts:
        # compute the center of the contour
        if cv2.contourArea(c) < 1000:
            continue
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        # draw the contour and center of the shape on the image
        cv2.drawContours(image_copy, [c], -1, (0, 255, 0), 2)
        cv2.circle(image_copy, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(image_copy, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
    # Display the resulting frame
    cv2.imshow('frame', cv_image)
    cv2.imshow('contours', image_copy)
    cv2.waitKey(1)