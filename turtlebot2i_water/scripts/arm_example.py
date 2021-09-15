


# Move the gripper to the neutral position
        rospy.loginfo("Set Gripper: Neutral " + str(self.gripper_neutral) )
        gripper.set_joint_value_target(self.gripper_neutral)
        if gripper.go() != True:
            rospy.logwarn("  Go failed")
        rospy.sleep(2)