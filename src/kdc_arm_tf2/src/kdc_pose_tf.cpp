#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void arucoPoseCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "duo3d_camera";
  transformStamped.child_frame_id = "board1";

  transformStamped.transform = msg->transform;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "kdc_tf2_listener");

  ros::NodeHandle node;

  ros::Subscriber sub;

  sub = node.subscribe("/ar_single_board/transform", 10, &arucoPoseCallback);

  ros::spin();
  return 0;
};


// void tfCallback(const geometry_msgs::TransformStamped& transformStamped){
//   static tf2_ros::TransformBroadcaster br;

//   tf2_ros::Buffer tfBuffer;
//   tf2_ros::TransformListener tfListener(tfBuffer);

//   transformStamped = tfBuffer.lookupTransform("board1", "base_link", ros::Time(0));
  
//   br.sendTransform(transformStamped);
// }

// int main(int argc, char** argv){
//   ros::init(argc, argv, "kdc_tf2_listener");

//   ros::NodeHandle node;

//   ros::Subscriber sub = node.subscribe("/ar_single_board/transform", 10 &tfCallback);

//   ros::Publisher board_tf =
//     node.advertise<geometry_msgs::TransformStamped>("kdc_arm", 10);

//   // tf2_ros::Buffer tfBuffer;
//   // tf2_ros::TransformListener tfListener(tfBuffer);

//   // ros::Rate rate(10.0);
//   // while (node.ok()){
//   //   geometry_msgs::TransformStamped transformStamped;
//   //   try{
//   //     transformStamped = tfBuffer.lookupTransform("board1", "base_link",
//   //                              ros::Time(0));
//   //   }
//   //   catch (tf2::TransformException &ex) {
//   //     ROS_WARN("%s",ex.what());
//   //     ros::Duration(1.0).sleep();
//   //     continue;
//   //   }

//   //   board_tf.publish(transformStamped);

//     rate.sleep();
//   }
//   return 0;
// };