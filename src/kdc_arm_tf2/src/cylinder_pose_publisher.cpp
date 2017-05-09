#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <list>



int main(int argc, char** argv){
  ros::init(argc, argv, "kdc_point_publisher");
 
  ros::NodeHandle node;

  ros::Publisher boardPoint =
    node.advertise<geometry_msgs::PointStamped>("kdc/board_point", 10);

  ros::Publisher cameraPoint =
    node.advertise<geometry_msgs::PointStamped>("kdc/camera_point", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);

  geometry_msgs::TransformStamped transformStampedBoard;
  geometry_msgs::TransformStamped transformStampedCamera;


  // Run publisher loop
  while (node.ok()){

    try{
      ros::Time now = ros::Time::now();
      transformStampedCamera = tfBuffer.lookupTransform("topPlate_pan1_link", "duo3d_camera",
                               now, ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    try{
      ros::Time now = ros::Time::now();
      transformStampedBoard = tfBuffer.lookupTransform("topPlate_pan1_link", "board1",
                               now, ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::PointStamped board_point;
    geometry_msgs::PointStamped board_point_filtered;
    geometry_msgs::PointStamped camera_point;

    board_point.header.stamp = ros::Time::now();
    board_point.header.frame_id = "topPlate_pan1_link";
    board_point.point.x = transformStampedBoard.transform.translation.x;
    board_point.point.y = transformStampedBoard.transform.translation.y;
    board_point.point.z = transformStampedBoard.transform.translation.z;

    camera_point.header.stamp = ros::Time::now();
    camera_point.header.frame_id = "topPlate_pan1_link";
    camera_point.point.x = transformStampedCamera.transform.translation.x;
    camera_point.point.y = transformStampedCamera.transform.translation.y;
    camera_point.point.z = transformStampedCamera.transform.translation.z;

    boardPoint.publish(board_point);
    cameraPoint.publish(camera_point);

    rate.sleep();
  }
  return 0;
};
