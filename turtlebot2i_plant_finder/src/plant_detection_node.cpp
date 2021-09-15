#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

namespace plant_detection_node
{

class PlantDetector
{
private:
  ros::NodeHandle node_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::StampedTransform tf_to_publish_;

  tf::TransformListener listener_;

  tf::Quaternion destination_rotation_[4];
  tf::Vector3 destination_position_[4];

  visualization_msgs::MarkerArray plant_rviz_markers_;

  ros::Publisher vis_pub_;

  tf::Transform plant_transforms_[4];

  double plant_offset_x_;

public:
  PlantDetector()
  {
    addMarker( &plant_rviz_markers_, 0, 1.0, 0.0, 0.0 );
    addMarker( &plant_rviz_markers_, 1, 0.0, 1.0, 0.0 );
    addMarker( &plant_rviz_markers_, 2, 0.0, 0.0, 1.0 );
    addMarker( &plant_rviz_markers_, 3, 1.0, 0.0, 0.0 );

    destination_position_[0].setZ( 0.0 );
    destination_position_[1].setZ( 0.0 );
    destination_position_[2].setZ( 0.0 );
    destination_position_[3].setZ( 0.0 );

    vis_pub_ = node_.advertise<visualization_msgs::MarkerArray>( "plant_visualization_marker", 0 );

    //Load parameters
    node_.param<double>("plant_offset_x", plant_offset_x_, 0.5);
  }

  void lookupPlants()
  {
    tf::StampedTransform transform;
    tf::StampedTransform displace_transform;

    tf::Vector3 offset_vector;
    offset_vector.setX( plant_offset_x_ ); //Offset destination in front of detected object

    displace_transform.setOrigin( offset_vector );
    
    try
    {
      listener_.lookupTransform("/map", "/object_1", ros::Time(0), transform);
      double angle = transform.getRotation().getAngle(); 
      transform *= displace_transform; //Displace destination and marker

      destination_position_[0] = transform.getOrigin();
      plant_rviz_markers_.markers[0].pose.position.x = destination_position_[0].x();
      plant_rviz_markers_.markers[0].pose.position.y = destination_position_[0].y();
      plant_rviz_markers_.markers[0].pose.position.z = destination_position_[0].z();
      plant_rviz_markers_.markers[0].color.a = 1.0;

      plant_transforms_[0].setOrigin( tf::Vector3(destination_position_[0].x(), destination_position_[0].y(), 0.0) );
      plant_transforms_[0].setRotation(tf::createQuaternionFromYaw(angle) * tf::createQuaternionFromYaw(3.14159));
      
      destination_rotation_[0] = plant_transforms_[0].getRotation();
      plant_rviz_markers_.markers[0].pose.orientation.x = destination_rotation_[0].getX();
      plant_rviz_markers_.markers[0].pose.orientation.y = destination_rotation_[0].getY();
      plant_rviz_markers_.markers[0].pose.orientation.z = destination_rotation_[0].getZ();
      plant_rviz_markers_.markers[0].pose.orientation.w = destination_rotation_[0].getW();

      ROS_INFO_STREAM_THROTTLE( 5, "Plant 1 Visible" );
    }
    catch (tf::TransformException &ex)
    {
      //ROS_ERROR_THROTTLE( 0.5, "%s",ex.what());
    }
    try
    {
      listener_.lookupTransform("/map", "/object_2", ros::Time(0), transform);
      double angle = transform.getRotation().getAngle();
      transform *= displace_transform;

      destination_position_[1] = transform.getOrigin();
      plant_rviz_markers_.markers[1].pose.position.x = destination_position_[1].x();
      plant_rviz_markers_.markers[1].pose.position.y = destination_position_[1].y();
      plant_rviz_markers_.markers[1].pose.position.z = destination_position_[1].z();
      plant_rviz_markers_.markers[1].color.a = 1.0;

      plant_transforms_[1].setOrigin( tf::Vector3(destination_position_[1].x(), destination_position_[1].y(), 0.0) );
      plant_transforms_[1].setRotation(tf::createQuaternionFromYaw(angle) * tf::createQuaternionFromYaw(3.14159));

      destination_rotation_[1] = plant_transforms_[1].getRotation();
      plant_rviz_markers_.markers[1].pose.orientation.x = destination_rotation_[1].getX();
      plant_rviz_markers_.markers[1].pose.orientation.y = destination_rotation_[1].getY();
      plant_rviz_markers_.markers[1].pose.orientation.z = destination_rotation_[1].getZ();
      plant_rviz_markers_.markers[1].pose.orientation.w = destination_rotation_[1].getW();

      ROS_INFO_STREAM_THROTTLE( 5, "Plant 2 Visible" );
    }
    catch (tf::TransformException &ex)
    {
      //ROS_ERROR("%s",ex.what());
    }
    try
    {
      listener_.lookupTransform("/map", "/object_3", ros::Time(0), transform);
      double angle = transform.getRotation().getAngle();
      transform *= displace_transform;

      destination_position_[2] = transform.getOrigin();
      plant_rviz_markers_.markers[2].pose.position.x = destination_position_[2].x();
      plant_rviz_markers_.markers[2].pose.position.y = destination_position_[2].y();
      plant_rviz_markers_.markers[2].pose.position.z = destination_position_[2].z();
      plant_rviz_markers_.markers[2].color.a = 1.0;

      plant_transforms_[2].setOrigin( tf::Vector3(destination_position_[2].x(), destination_position_[2].y(), 0.0) );
      plant_transforms_[2].setRotation(tf::createQuaternionFromYaw(angle) * tf::createQuaternionFromYaw(3.14159));

      destination_rotation_[2] = plant_transforms_[2].getRotation();
      plant_rviz_markers_.markers[2].pose.orientation.x = destination_rotation_[2].getX();
      plant_rviz_markers_.markers[2].pose.orientation.y = destination_rotation_[2].getY();
      plant_rviz_markers_.markers[2].pose.orientation.z = destination_rotation_[2].getZ();
      plant_rviz_markers_.markers[2].pose.orientation.w = destination_rotation_[2].getW();

      ROS_INFO_STREAM_THROTTLE( 5, "Plant 3 Visible" );
    }
    catch (tf::TransformException &ex)
    {
      //ROS_ERROR("%s",ex.what());
    }
    try
    {
      listener_.lookupTransform("/map", "/object_4", ros::Time(0), transform);
      double angle = transform.getRotation().getAngle();
      transform *= displace_transform;

      destination_position_[3] = transform.getOrigin();
      plant_rviz_markers_.markers[3].pose.position.x = destination_position_[3].x();
      plant_rviz_markers_.markers[3].pose.position.y = destination_position_[3].y();
      plant_rviz_markers_.markers[3].pose.position.z = destination_position_[3].z();
      plant_rviz_markers_.markers[3].color.a = 1.0;

      plant_transforms_[3].setOrigin( tf::Vector3(destination_position_[3].x(), destination_position_[3].y(), 0.0) );
      plant_transforms_[3].setRotation(tf::createQuaternionFromYaw(angle) * tf::createQuaternionFromYaw(3.14159));

      destination_rotation_[3] = plant_transforms_[3].getRotation();
      plant_rviz_markers_.markers[3].pose.orientation.x = destination_rotation_[3].getX();
      plant_rviz_markers_.markers[3].pose.orientation.y = destination_rotation_[3].getY();
      plant_rviz_markers_.markers[3].pose.orientation.z = destination_rotation_[3].getZ();
      plant_rviz_markers_.markers[3].pose.orientation.w = destination_rotation_[3].getW();

      ROS_INFO_STREAM_THROTTLE( 5, "Plant 4 Visible" );
    }
    catch (tf::TransformException &ex)
    {
      //ROS_ERROR("%s",ex.what());
    }

    if ( destination_position_[0].z() == 0.0 )
    {
      ROS_WARN_STREAM_THROTTLE( 5, "Plant 1 not identified." );
    }
    else
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(plant_transforms_[0], ros::Time::now(), "map", "plant_1"));
      ROS_INFO_STREAM_THROTTLE( 5, "Plant 1 Area X " << destination_position_[0].x() << " Y " << destination_position_[0].y() << " Z " << destination_position_[0].z() );
    } 
    if ( destination_position_[1].z() == 0.0 )
    {
      ROS_WARN_STREAM_THROTTLE( 5, "Plant 2 not identified." );
    }
    else 
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(plant_transforms_[1], ros::Time::now(), "map", "plant_2"));
      ROS_INFO_STREAM_THROTTLE( 5, "Plant 2 Area X " << destination_position_[1].x() << " Y " << destination_position_[1].y() << " Z " << destination_position_[1].z() );
    }
    if ( destination_position_[2].z() == 0.0 )
    {
      ROS_WARN_STREAM_THROTTLE( 5, "Plant 3 not identified." );
    }
    else
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(plant_transforms_[2], ros::Time::now(), "map", "plant_3"));
      ROS_INFO_STREAM_THROTTLE( 5, "Plant 3 Area X " << destination_position_[2].x() << " Y " << destination_position_[2].y() << " Z " << destination_position_[2].z() );
    }
    if ( destination_position_[3].z() == 0.0 )
    {
      ROS_WARN_STREAM_THROTTLE( 5, "Plant 4 not identified." );
    }
    else
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(plant_transforms_[3], ros::Time::now(), "map", "plant_4"));
      ROS_INFO_STREAM_THROTTLE( 5, "Plant 4 Area X " << destination_position_[3].x() << " Y " << destination_position_[3].y() << " Z " << destination_position_[3].z() );
    }
    vis_pub_.publish( plant_rviz_markers_ );
  }

private:

  void addMarker( visualization_msgs::MarkerArray * p_array, int id, double red, double green, double blue )
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "turtlebot_plants";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.0; // Invisible until identified
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    p_array->markers.push_back( marker );
  }

};

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plant_detection_node");

  plant_detection_node::PlantDetector plant_detector;

  ros::Rate rate(5.0);
  while (ros::ok())
  {
    ros::spinOnce();
    plant_detector.lookupPlants();
    rate.sleep();
  }
  return 0;
};
