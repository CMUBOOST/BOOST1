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
    pub_ = n_.advertise<geometry_msgs::Twist>("/camera_stitch/cmd_vel", 1);

    //Topic you want to subscribe
    //sub_ = n_.subscribe("/joint_states", 1, &SubscribeAndPublish::callback, this);
    sub_ = n_.subscribe("/wheel_encoder/odom", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const nav_msgs::Odometry::ConstPtr& robotLoc)//sensor_msgs::JointState::ConstPtr& jointStatePtr)
  {

    //ROS_DEBUG_STREAM("Location: " << robotLoc->pose.pose.position.x);
    //ROS_INFO_STREAM("I'm in the callback");
    if (globalFlag == 0) {
        initialLocation = robotLoc->pose.pose.position.x;
        globalFlag = 1;
    }

    float currentLocation = robotLoc->pose.pose.position.x;
    //float Front_Left_Drive = jointStatePtr->position[0];
    //float Front_Right_Drive = jointStatePtr->position[1];
    geometry_msgs::Twist output;

    if((currentLocation-initialLocation)<0.5){
        output.linear.x = 0.33;
        output.linear.y = 0.0;
        output.linear.z = 0.0;
        output.angular.x = 0.0;
        output.angular.y = 0.0;
        output.angular.z = 0.0;
    }
    else {
        output.linear.x = 0.0;
        output.linear.y = 0.0;
        output.linear.z = 0.0;
        output.angular.x = 0.0;
        output.angular.y = 0.0;
        output.angular.z = 0.0;
    }


    //ROS_DEBUG_STREAM("Front_Left_Drive: " << Front_Left_Drive << ", Front_Right_Drive: " << Front_Right_Drive);
    //std::cout << "Front_Left_Drive: " << Front_Left_Drive << ", Front_Right_Drive: " << Front_Right_Drive << std::endl;

    pub_.publish(output);
    //ROS_INFO_STREAM("Just published!");
    
  }



private:
  //float x_estimate;
  //float y_estimate;
  //std::string direction;
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

/*
bool servo(hebi::ArmServo::Request  &req,
         hebi::ArmServo::Response &res)
{

    SubscribeAndPublish SAPObject;

    SAPObject.getDestination(req.arm_servo_req_x, req.arm_servo_req_y, req.arm_servo_req_jointAngle);

    ros::Rate loop_rate(100);

    //Do stuff

    res.arm_servo_resp = req.arm_servo_req_jointAngle;

    return true;
}
*/

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "move_base");

  ros::NodeHandle n;

  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  //ros::ServiceServer service = n.advertiseService("move_base", servo);

  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}



/*
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const SUBSCRIBED_MESSAGE_TYPE& input)
  {
    PUBLISHED_MESSAGE_TYPE output;
    //.... do something with the input and generate the output...
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
*/