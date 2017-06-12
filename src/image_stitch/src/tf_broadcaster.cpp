#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"

class Broadcast
{
public:
	void callback(const nav_msgs::Odometry::ConstPtr& msg);

private:
	tf::TransformBroadcaster broadcaster;
};


void Broadcast::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO_STREAM("x:" << msg->pose.pose.position.x);
  //ROS_INFO_STREAM("x:" << msg->pose.pose.orientation.x);
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w), 
          tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)),
          ros::Time::now(),"map","base_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;
  Broadcast* pBroad = new Broadcast;
  ros::Rate r(50);

  ros::Subscriber sub = n.subscribe("/wheel_encoder/odom", 1000, &Broadcast::callback, pBroad);

  while(n.ok()){
    r.sleep();
    ros::spin();
  }
}
