#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <log4cxx/logger.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_NAN");
  ros::NodeHandle n;
  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  //ros::ServiceServer service = n.advertiseService("arm_NAN", sensNANs);
  //ROS_INFO("Ready to send NANs.");
  //ros::spin();

  ros::Publisher NAN_pub = n.advertise<sensor_msgs::JointState>("joint_commands", 100);
  sensor_msgs::JointState msg;
  msg.position.push_back(NAN);
  msg.position.push_back(NAN);
  msg.position.push_back(NAN);
  msg.velocity.push_back(0.0);
  msg.velocity.push_back(0.0);
  msg.velocity.push_back(0.0);
  ros::Rate loop_rate(100);
  int count = 0;

  while(count<100)
  {
      NAN_pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
  }

  return 0;
}
