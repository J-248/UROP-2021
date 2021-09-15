#! /usr/bin/env python
# Inspired by https://www.theconstructsim.com/wall-follower-algorithm/

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge=CvBridge()

# Image resolution 640 x 480
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
all_regions = [A,B,C,D,E,F,G,H]

"""
A|B|C|D
E|F|G|H
"""

def block(cv_img, limit):
    return np.array(cv_img[limit[3]:limit[2], limit[1]:limit[0]])

def block_average(block, limit):
    return np.sum(block)/((limit[0] - limit[1])*(limit[2] - limit[3]))

def check_location(x, y, lst):
    for region in lst:
        if region[1] <= x < region[0] and region[3] <= y < region[2]:
            return region
    return False

def narrow_depth(x, y):
    x1 = np.where(x+60 > res_x, res_x, x+60)
    x0 = np.where(x-50 <0, 0, x-50)
    y1 = np.where(y+60 > res_y, res_y, x+60)
    y0 = np.where(y-50 <0, 0, y-50)
    return [x1, x0, y1, y0]

depth = 850
p_depth = 50
av_n = None
N_ = None
pub_ = None
regions_ = {
    'right':  0,
    'front':  0,
    'left':   0,
    'plant':  0,
}

state_ = 0
state_dict_ = {
    0: 'find the wall', # Moves straight + right
    1: 'turn left', # Avoid obstacle
    2: 'follow the wall', # Moves straight if an obstacle is on the right
    3: 'stop'
}

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        state_ = state

def callback_rgb(img):
    global N_
    cv_image = bridge.imgmsg_to_cv2(img, "passthrough")
    cv_image = cv2.blur(cv_image, (5,5)) # Averaging pixels in a 5x5 kernel to remove noise
    cv_image = cv2.GaussianBlur(cv_image,(5,5),0) # Further smoothing using Gaussian deviation
    hsvFrame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # Obtaining the HSV(hue-saturation-value)

    # Setting range for green colour
    green_lower = np.array([25, 52, 72], np.uint8)
    green_upper = np.array([102, 255, 255], np.uint8)
    # Creating mask for green
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 
    green_mask = cv2.dilate(green_mask, np.ones((5, 5), "uint8"))
    # Finding contours of green areas
    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    for item, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 500):
                x, y, w, h = cv2.boundingRect(contour)
                M = cv2.moments(contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv_image = cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                #Adding text, contour & centroid
                cv2.putText(cv_image, "Green Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0))
                cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
                cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(cv_image, "center", (cX - 20, cY - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                #print("green", cX, cY)
                location = check_location(cX, cY, all_regions)
                if location is not False:
                    if location in (B,C,F,G):
                        change_state(3)
                        N_ = narrow_depth(cX, cY)
                         

def callback_depth(img):
    global regions_
    global av_n
    cv_image = bridge.imgmsg_to_cv2(img, "passthrough")
    cv_image = cv2.blur(cv_image, (5,5)) # Averaging pixels in a 5x5 kernel to remove noise
    cv_image = cv2.GaussianBlur(cv_image,(5,5),0) # Further smoothing using Gaussian deviation
  

    a = block(cv_image, A)
    b = block(cv_image, B)
    c = block(cv_image, C)
    d = block(cv_image, D)
    e = block(cv_image, E)
    f = block(cv_image, F)
    g = block(cv_image, G)
    h = block(cv_image, H)
    
    av_a = block_average(a, A) # top left
    av_b = block_average(b, B) # top mid left
    av_c = block_average(c, C) # top mid right
    av_d = block_average(d, D) # top right
    av_e = block_average(e, E) # bottom left
    av_f = block_average(f, F) # bottom mid left
    av_g = block_average(g, G) # bottom mid right
    av_h = block_average(h, H) # bottom right
    
    N = N_
    
    try:
        n = block(cv_image, N)
        av_n = block_average(n, N)
    except:
        av_n = 1000
    #av_n = av_n

    regions_ = {
        'right':     min(av_d, av_h),
        'front':     min(av_b, av_c, av_f, av_g),
        'left':      min(av_a, av_e),
        'plant':     av_n,
    }
    take_action()


def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0


    if regions['front'] > depth and regions['left'] > depth and regions['right'] > depth and regions['plant'] > p_depth:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < depth and regions['left'] > depth and regions['right'] > depth and regions['plant'] > p_depth:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > depth and regions['left'] > depth and regions['right'] < depth and regions['plant'] > p_depth:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > depth and regions['left'] < depth and regions['right'] > depth and regions['plant'] > p_depth:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < depth and regions['left'] > depth and regions['right'] < depth and regions['plant'] > p_depth:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < depth and regions['left'] < depth and regions['right'] > depth and regions['plant'] > p_depth:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < depth and regions['left'] < depth and regions['right'] < depth and regions['plant'] > p_depth:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > depth and regions['left'] < depth and regions['right'] < depth and regions['plant'] > p_depth:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        change_state(3)
        # state_description = 'unknown case'
        # rospy.loginfo(regions)



def main():
    global pub_
    
    rospy.init_node('green_follower')
    pub_ = rospy.Publisher('~cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/camera/depth/image_raw', Image, callback_depth)
    subrgb = rospy.Subscriber('/camera/rgb/image_raw', Image, callback_rgb)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()


        # Velocity smoothing (untested)
        # target_speed = speed * x
        # target_turn = turn * th

        # if target_speed > control_speed:
        #     control_speed = min( target_speed, control_speed + 0.02 )
        # elif target_speed < control_speed:
        #     control_speed = max( target_speed, control_speed - 0.02 )
        # else:
        #     control_speed = target_speed

        # if target_turn > control_turn:
        #     control_turn = min( target_turn, control_turn + 0.1 )
        # elif target_turn < control_turn:
        #     control_turn = max( target_turn, control_turn - 0.1 )
        # else:
        #     control_turn = target_turn

        # twist = Twist()
        # twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
        # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
        # pub_move.publish(twist)


        if state_ == 0:
            msg.linear.x = 0.1
            msg.angular.z = -0.1
        elif state_ == 1:
            msg.angular.z = 0.7
        elif state_ == 2:
            msg.linear.x = 0.1
            msg.angular.z = 0.05
        elif state_ == 3:
            msg.linear.x = 0
            msg.angular.z = 0
            rospy.sleep(1)
            pass
        else:
            rospy.logerr('Unknown state!')

        # if state_ == 0:
        #     print("finding wall")
        # elif state_ == 1:
        #     print("turning left")
        # elif state_ == 2:
        #     print("following wall")
        # elif state_ == 3:
        #     print("plant detected", av_n)
        #     rospy.sleep(1)
        #     pass
        # else:
        #     rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()