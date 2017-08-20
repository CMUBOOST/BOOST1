#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

using namespace sensor_msgs;
using namespace message_filters;

//void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)

//void callback(const boost::shared_ptr<const sensor_msgs::JointState>& image1, const boost::shared_ptr<const nav_msgs::Odometry>& image2)
void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& image1, const boost::shared_ptr<const nav_msgs::Odometry>& image2)
//void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& image1, const boost::shared_ptr<const sensor_msgs::PointCloud2>& image2)
//void callback(const boost::shared_ptr<const geometry_msgs::Twist>& image1, const boost::shared_ptr<const nav_msgs::Odometry>& image2)
{
  // Solve all of perception here...
  std::cout << "i'm in here!!" << std::endl;
  std::cerr << "i'm in here!!" << std::endl;
  ROS_INFO_STREAM("I'm here!");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stitch_node");

  ros::NodeHandle nh;

  //ros::Rate loop_rate(100);

  message_filters::Subscriber<sensor_msgs::PointCloud2> image1_sub(nh, "/duo3d_camera/points2", 1); 
  //message_filters::Subscriber<sensor_msgs::JointState> image1_sub(nh, "/joint_states", 1);
  message_filters::Subscriber<nav_msgs::Odometry> image2_sub(nh, "/wheel_encoder/odom", 1);
  //message_filters::Subscriber<sensor_msgs::PointCloud2> image2_sub(nh, "/multisense/image_points2_color", 1);

  //typedef sync_policies::ApproximateTime<sensor_msgs::JointState, nav_msgs::Odometry> MySyncPolicy;
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  //typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;  
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image1_sub, image2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  //while(ros::ok()) {
  //    ros::spinOnce();
  //    loop_rate.sleep();
  //}

  return 0;
}
