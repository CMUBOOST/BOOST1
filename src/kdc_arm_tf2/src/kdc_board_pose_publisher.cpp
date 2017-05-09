#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <list>



int main(int argc, char** argv){
  ros::init(argc, argv, "kdc_point_publisher");

  const double NUM_SAMPLES = 10;
  const int NUM_S_int = 10;
 
  ros::NodeHandle node;

  ros::Publisher boardPoint =
    node.advertise<geometry_msgs::PointStamped>("kdc/board_point", 10);
  // ros::Publisher boardPoint_filtered =
  //   node.advertise<geometry_msgs::PointStamped>("kdc/board_point_filtered", 10);
  ros::Publisher cameraPoint =
    node.advertise<geometry_msgs::PointStamped>("kdc/camera_point", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);

  // Initialize array of samples
  // double xBoard [NUM_S_int] = { };
  // double yBoard [NUM_S_int] = { };
  // double zBoard [NUM_S_int] = { };
  // std::list<double> xBoard;
  // std::list<double> yBoard;
  // std::list<double> zBoard;
  // double xBoard[10] = {};
  // double yBoard[10] = {};
  // double zBoard[10] = {};

  // Populate array of board points first before publishing points
  geometry_msgs::TransformStamped transformStampedBoard;
  geometry_msgs::TransformStamped transformStampedCamera;
  // for (int i = 0; i < NUM_SAMPLES; i++)
  //       {
  //   try{
  //     ros::Time now = ros::Time::now();
  //     transformStampedBoard = tfBuffer.lookupTransform("topPlate_pan1_link", "board1",
  //                              now, ros::Duration(0.5));
  //   }
  //   catch (tf2::TransformException &ex) {
  //     ROS_WARN("%s",ex.what());
  //     ros::Duration(1.0).sleep();
  //     continue;
  //   }

  //   // xBoard[i] = transformStampedBoard.transform.translation.x;
  //   // yBoard[i] = transformStampedBoard.transform.translation.y;
  //   // zBoard[i] = transformStampedBoard.transform.translation.z;
  //   xBoard.push_back(transformStampedBoard.transform.translation.x);
  //   yBoard.push_back(transformStampedBoard.transform.translation.y);
  //   zBoard.push_back(transformStampedBoard.transform.translation.z);

  // }


  // Run publisher loop
  while (node.ok()){

    // geometry_msgs::TransformStamped transformStampedBoard;
    // geometry_msgs::TransformStamped transformStampedCamera;

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

    // Calculate moving average of each coordinate of board position
    // xBoard.push_back(transformStampedBoard.transform.translation.x);
    // xBoard.pop_front();
    // yBoard.push_back(transformStampedBoard.transform.translation.y);
    // yBoard.pop_front();
    // zBoard.push_back(transformStampedBoard.transform.translation.z);
    // zBoard.pop_front();

    // double xSum = 0;
    // double ySum = 0;
    // double zSum = 0;

    // std::list<double>::const_iterator cii;
    // for(cii=xBoard.begin(); cii!=xBoard.end(); cii++)
    // {
    //   xSum += *cii;
    // }
    // for(cii=yBoard.begin(); cii!=yBoard.end(); cii++)
    // {
    //   ySum += *cii;
    // }
    // for(cii=zBoard.begin(); cii!=zBoard.end(); cii++)
    // {
    //   zSum += *cii;
    // }

    // board_point_filtered.header.stamp = ros::Time::now();
    // board_point_filtered.header.frame_id = "topPlate_pan1_link";
    // board_point_filtered.point.x = xSum / double(xBoard.size());
    // board_point_filtered.point.y = ySum / double(yBoard.size());
    // board_point_filtered.point.z = zSum / double(zBoard.size());

    camera_point.header.stamp = ros::Time::now();
    camera_point.header.frame_id = "topPlate_pan1_link";
    camera_point.point.x = transformStampedCamera.transform.translation.x;
    camera_point.point.y = transformStampedCamera.transform.translation.y;
    camera_point.point.z = transformStampedCamera.transform.translation.z;

    boardPoint.publish(board_point);
    // boardPoint_filtered.publish(board_point_filtered);
    cameraPoint.publish(camera_point);

    rate.sleep();
  }
  return 0;
};