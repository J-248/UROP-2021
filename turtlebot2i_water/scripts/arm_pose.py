#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


def main():
    # Initialise moveit_commander and a rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_pose', anonymous=True)

    robot = moveit_commander.RobotCommander() # Outer-level interface to robot
    scene = moveit_commander.PlanningSceneInterface() # Interface to world surrounding robot
    group = moveit_commander.MoveGroupCommander("pincher_arm") # Interface to plan and execute arm motion
    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    
    group.clear_pose_targets()
    group_variable_values = group.get_current_joint_values()
    #print("============ Joint values: ", group_variable_values)

    group_variable_values[0] = -1.5
    group_variable_values[1] = -1.5
    group_variable_values[2] = -1.0
    group_variable_values[3] = 0.5
    group_variable_values[4] = 0.5
    #print("============ Joint values: ", group_variable_values)
    

    # group.set_joint_value_target(group_variable_values)
    group.set_goal_joint_tolerance(0.1)
    #group.plan()
    group.go(group_variable_values, wait=True)
    
    rospy.sleep(3)
    group.stop()

    # Move to a pose goal
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.0
    # pose_goal.position.x = 0.1
    # pose_goal.position.y = 0.1
    # pose_goal.position.z = 0.1
    # group.set_pose_target(pose_goal)
    # group.go(pose_goal, wait=True)
    # rospy.sleep(3)
    # group.stop()
    # group.clear_pose_targets()

    # Move to a named pose
    # group.set_named_target("pose_parked")
    # pincher_arm = [pose_parked, pose_grasping, aim_nozzle]
    # pincher_gripper = [gripper_open, gripper_closed]

    moveit_commander.roscpp_shutdown()
    return


if __name__ == '__main__':
    main()

#  Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
# [ INFO] [1629890389.624435919]: Planning attempt 1 of at most 1
# [ INFO] [1629890389.625843605]: Planner configuration 'pincher_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
# [ INFO] [1629890389.626336669]: RRTConnect: Starting planning with 1 states already in datastructure
# [ INFO] [1629890389.642021207]: RRTConnect: Created 5 states (2 start + 3 goal)
# [ INFO] [1629890389.642198635]: Solution found in 0.016049 seconds
# [ INFO] [1629890389.688945799]: SimpleSetup: Path simplification took 0.046624 seconds and changed from 4 to 2 states
# [INFO] [1629890389.712875]: arm_controller: Action goal recieved.
# [INFO] [1629890389.713164]: Executing trajectory