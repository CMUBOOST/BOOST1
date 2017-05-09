#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

void PoseCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "duo3d_camera";
  transformStamped.child_frame_id = "board1";

  transformStamped.transform.translation.x = msg->point.x;
  transformStamped.transform.translation.y = msg->point.y;
  transformStamped.transform.translation.z = msg->point.z;
  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;
  //transformStamped.transform = msg->transform;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "kdc_tf2_listener");

  ros::NodeHandle node;

  ros::Subscriber sub;

  sub = node.subscribe("/duo3d_camera/points_centroid", 10, &PoseCallback);

  ros::spin();
  return 0;
};
