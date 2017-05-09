#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

const double DEADBAND = 0.01;  // Prevent oscillation around 0 due to pose estimation error, adjust as needed

double boardCallback(const geometry_msgs::PointStamped::ConstPtr& msg1)
{
  double boardZ;
  boardZ = msg1->points[1];
  return(boardZ);
}

double cameraCallback(const geometry_msgs::PointStamped::ConstPtr& msg2)
{
  double cameraZ;
  cameraZ = msg1->points[1];
  return(cameraZ);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "kdc_z_follower");

  ros::NodeHandle node;

  ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("joint_commands", 10);

  ros::Subscriber sub_board_point = node.subscribe("kdc/board_point", 1, &boardCallback);
  ros::Subscriber sub_camera_point = node.subscribe("kdc/camera_point", 1, &boardCallback);

  while (node.ok()){

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";

    if (abs(sub_board_point.point.z - sub_camera_point.point.z) > DEADBAND) {
      z_vel_com = 0;
    } else {
      z_vel_com = sub_camera_point.point.z - sub_board_point.point.z;
    }

    std::cout << z_vel_com << std::endl;


    ros::spin();
  }

  // ros::NodeHandle node;

  // ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_commands", 10);


  //   geometry_msgs::PointStamped board_point;
  //   geometry_msgs::PointStamped camera_point;

  //   board_point.header.stamp = ros::Time::now();
  //   board_point.header.frame_id = "base_link";
  //   board_point.point.x = transformStampedBoard.transform.translation.x;
  //   board_point.point.y = transformStampedBoard.transform.translation.y;
  //   board_point.point.z = transformStampedBoard.transform.translation.z;
   
  //   camera_point.header.stamp = ros::Time::now();
  //   camera_point.header.frame_id = "base_link";
  //   camera_point.point.x = transformStampedCamera.transform.translation.x;
  //   camera_point.point.y = transformStampedCamera.transform.translation.y;
  //   camera_point.point.z = transformStampedCamera.transform.translation.z;

  //   boardPoint.publish(board_point);
  //   cameraPoint.publish(camera_point);

  //   rate.sleep();
  // }



  return 0;
};

