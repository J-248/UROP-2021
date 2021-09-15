#! /usr/bin/env python
# Inspired by https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
  
# Capturing video through webcam
bridge = CvBridge()
webcam = cv2.VideoCapture(0)
  
# Start a while loop
def get_colour(img):  
    cv_image = bridge.imgmsg_to_cv2(img, "passthrough") # Convert ROS image to CV image   
    cv_image = cv2.blur(cv_image, (5,5))
    cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
    hsvFrame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # Convert to HSV(hue-saturation-value)

  
    # Set range for green color and define mask
    green_lower = np.array([25, 52, 72], np.uint8)
    green_upper = np.array([102, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
      
    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")
      
    # For green color
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(cv_image, cv_image,
                                mask = green_mask)
 
  
    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)[-2:]
 
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            cv_image = cv2.rectangle(cv_image, (x, y), 
                                       (x + w, y + h),
                                       (0, 255, 0), 2)
              
            cv2.putText(cv_image, "Green Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        1.0, (0, 255, 0))
            M = cv2.moments(contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # draw the contour and center of the shape on the image
            cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
            cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
            cv2.putText(cv_image, "center", (cX - 20, cY - 20),
    		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
 
              
    # Program Termination
    cv2.imshow("Colour & Centroid", cv_image)
    cv2.waitKey(1)
    return

def main():
    rospy.init_node('colour_listener')
    rospy.Subscriber('/camera/rgb/image_raw',Image,get_colour)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()