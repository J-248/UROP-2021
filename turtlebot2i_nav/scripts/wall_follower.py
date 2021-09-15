#! /usr/bin/env python
# Inspired by https://www.theconstructsim.com/wall-follower-algorithm/

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import math
from cv_bridge import CvBridge, CvBridgeError
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

"""
A|B|C|D
E|F|G|H
"""

def block(cv_img, limit):
    return np.array(cv_img[limit[3]:limit[2], limit[1]:limit[0]])

def block_average(block, limit):
    return np.sum(block)/((limit[0] - limit[1])*(limit[2] - limit[3]))

d = 850

pub_ = None
regions_ = {
    'right':  0,
    'front':  0,
    'left':   0,
}

state_ = 0
state_dict_ = {
    0: 'find the wall', # Moves straight + right
    1: 'turn left', # Avoid obstacle
    2: 'follow the wall', # Moves straight if an obstacle is on the right
}

def callback_image(img):
    global regions_
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

    av_a = block_average(a, A) # top left
    av_b = block_average(b, B) # top mid left
    av_c = block_average(c, C) # top mid right
    av_d = block_average(d, D) # top right
    av_e = block_average(e, E) # bottom left
    av_f = block_average(f, F) # bottom mid left
    av_g = block_average(g, G) # bottom mid right
    av_h = block_average(h, H) # bottom right

    regions_ = {
        'right':     min(av_d, av_h),
        'front':     min(av_b, av_c, av_f, av_g),
        'left':      min(av_a, av_e),
    }

    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        #print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    if regions['front'] > d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

def find_wall():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.1
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.7
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = 0.05
    return msg

def main():
    global pub_
    
    rospy.init_node('wall_follower')
    pub_ = rospy.Publisher('~cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/camera/depth/image_raw', Image, callback_image)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()



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
            msg = find_wall()
            #print("finding wall")
        elif state_ == 1:
            msg = turn_left()
            #print("turning left")
        elif state_ == 2:
            msg = follow_the_wall()
            #print("following wall")
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()