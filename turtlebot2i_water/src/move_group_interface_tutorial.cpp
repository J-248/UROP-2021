// https://github.com/ros-planning/moveit_tutorials/blob/indigo-devel/doc/pr2_tutorials/planning/src/move_group_interface_tutorial.cpp
// http://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/move_group_interface_tutorial.html

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  sleep(20.0);
  
  // BEGIN_TUTORIAL
  // 
  // Setup
  // ^^^^^
  // 
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("pincher_arm");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  


  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // // Planning to a Pose goal
  // // ^^^^^^^^^^^^^^^^^^^^^^^
  // // We can plan a motion for this group to a desired pose for the 
  // // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;
  group.setPoseTarget(target_pose1);


  // // Now, we call the planner to compute the plan
  // // and visualize it.
  // // Note that we are just planning, not asking move_group 
  // // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);
  group.move()
  // ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  // /* Sleep to give Rviz time to visualize the plan. */
  // sleep(5.0);

  // // Visualizing plans
  // // ^^^^^^^^^^^^^^^^^
  // // Now that we have a plan we can visualize it in Rviz.  This is not
  // // necessary because the group.plan() call we made above did this
  // // automatically.  But explicitly publishing plans is useful in cases that we
  // // want to visualize a previously created plan.
  // if (1)
  // {
  //   ROS_INFO("Visualizing plan 1 (again)");    
  //   display_trajectory.trajectory_start = my_plan.start_state_;
  //   display_trajectory.trajectory.push_back(my_plan.trajectory_);
  //   display_publisher.publish(display_trajectory);
  //   /* Sleep to give Rviz time to visualize the plan. */
  //   sleep(5.0);
  // }
  
  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active 
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is 
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.
 
  /* Uncomment below line when working with a real robot*/
  /* group.move() */

  // Planning to a joint-space goal 
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // // First get the current set of joint values for the group.
  // std::vector<double> group_variable_values;
  // group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  
  // // Now, let's modify one of the joints, plan to the new joint
  // // space goal and visualize the plan.
  // group_variable_values[0] = -1.0;  
  // group.setJointValueTarget(group_variable_values);
  // success = group.plan(my_plan);
  // group.move()
  // ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
  // /* Sleep to give Rviz time to visualize the plan. */
  // sleep(5.0);


  ros::shutdown();  
  return 0;
}