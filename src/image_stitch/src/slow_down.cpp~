#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <math.h>
#include <log4cxx/logger.h>


//int idleFlag = 0;
//int finalMoveFlag = 0;
//int counter;
int globalFlag = 0;
float initialLocation;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<nav_msgs::Odometry>("/odometry/filtered_imu_encoders_slow", 1);

    //Topic you want to subscribe
    //sub_ = n_.subscribe("/joint_states", 1, &SubscribeAndPublish::callback, this);
    sub_ = n_.subscribe("/odometry/filtered_imu_encoders", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const nav_msgs::Odometry::ConstPtr& robotLoc)//sensor_msgs::JointState::ConstPtr& jointStatePtr)
  {

    //ROS_DEBUG_STREAM("Front_Left_Drive: " << Front_Left_Drive << ", Front_Right_Drive: " << Front_Right_Drive);
    //std::cout << "Front_Left_Drive: " << Front_Left_Drive << ", Front_Right_Drive: " << Front_Right_Drive << std::endl;

    pub_.publish(robotLoc);
    //ROS_INFO_STREAM("Just published!");
    
  }



private:

  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish



int main(int argc, char **argv)
{

  ros::init(argc, argv, "slow_down");

  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  SubscribeAndPublish SAPObject;

  while(ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
