#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge=CvBridge()
res_x, res_y = 640, 480

def split(resolution_x, resolution_y, xn, yn):
    x_split, y_split = [], []
    for i in range(0, xn+1):
        x_split.append(resolution_x/xn * i)
    for j in range(0, yn+1):
        y_split.append(resolution_y/yn * j)    
    return x_split, y_split

splitted = split(res_x, res_y, 4, 2)

# [0 - xmax, 1 - xmin, 2 - ymax, 3 - ymin]
A = [splitted[0][1], splitted[0][0], splitted[1][1], splitted[1][0]]
B = [splitted[0][2], splitted[0][1], splitted[1][1], splitted[1][0]]
C = [splitted[0][3], splitted[0][2], splitted[1][1], splitted[1][0]]
D = [splitted[0][4], splitted[0][3], splitted[1][1], splitted[1][0]]
E = [splitted[0][1], splitted[0][0], splitted[1][2], splitted[1][1]]
F = [splitted[0][2], splitted[0][1], splitted[1][2], splitted[1][1]]
G = [splitted[0][3], splitted[0][2], splitted[1][2], splitted[1][1]]
H = [splitted[0][4], splitted[0][3], splitted[1][2], splitted[1][1]]

"""
A|B|C|D
E|F|G|H
"""

def block(cv_img, limit):
    return np.array(cv_img[limit[3]:limit[2], limit[1]:limit[0]])

def block_average(block, limit):
    return np.sum(block)/((limit[0] - limit[1])*(limit[2] - limit[3]))

def get_distance(img):
    cv_image = bridge.imgmsg_to_cv2(img, "passthrough")
    cv_image = cv2.blur(cv_image, (5,5))
    cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
    a = block(cv_image, A)
    b = block(cv_image, B)
    c = block(cv_image, C)
    d = block(cv_image, D)
    e = block(cv_image, E)
    f = block(cv_image, F)
    g = block(cv_image, G)
    h = block(cv_image, H)

    av_a = block_average(a, A)
    av_b = block_average(b, B)
    av_c = block_average(c, C)
    av_d = block_average(d, D)
    av_e = block_average(e, E)
    av_f = block_average(f, F)
    av_g = block_average(g, G)
    av_h = block_average(h, H)

    print(av_a, av_b, av_c, av_d, av_e, av_f, av_g, av_h)

    # Test to see location for block A
    # cv2.rectangle(cv_image, (E[0], E[2]), (E[1], E[3]), (20000), 3)
    # cv2.imshow("img", cv_image)
    # cv2.waitKey(1)

    rospy.sleep(1)
    return

def main():
    rospy.init_node('depth_listener')
    rospy.Subscriber('/camera/depth/image_raw',Image,get_distance)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    
    main()