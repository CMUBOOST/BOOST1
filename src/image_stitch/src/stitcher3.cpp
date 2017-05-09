#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

//ros::Publisher Velodyne_pub;

void callback(const geometry_msgs::Twist::ConstPtr& point_cloud2, const nav_msgs::Odometry::ConstPtr& laser_scan)
//void callback(const boost::shared_ptr<const geometry_msgs::Twist>& image2, const boost::shared_ptr<const nav_msgs::Odometry>& image1)
{
    ROS_ERROR("Enter Publish");
   // Velodyne_pub.publish(point_cloud2);
  //  Hokuyo_pub.publish(laser_scan);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "message_filter_node");
  ros::Time::init();
  ros::NodeHandle nh;
  ROS_INFO("start message filter");
  message_filters::Subscriber<sensor_msgs::PointCloud2> Velodyne_sub(nh, "/chatter", 1);
  message_filters::Subscriber<nav_msgs::Odometry> Hokuyo_sub(nh,"/odometry/filtered_imu_encoders", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), Velodyne_sub, Hokuyo_sub);
  ROS_INFO("about to start callback");
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  return 0;
}
