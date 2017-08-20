#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "hebi/Stage.h"
#include <sstream>
#include <math.h>
#include <log4cxx/logger.h>

#include "servoVars.hpp"

float destination;
int counter;
float stage_loc;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::JointState>("/joint_commands", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/joint_states", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const sensor_msgs::JointState::ConstPtr& jointStatePtr)
  {
    //ROS_INFO_STREAM("I'm in the callback");
    stage_loc = jointStatePtr->position[5];

    sensor_msgs::JointState output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "";

    output.position.push_back(NAN);
    output.position.push_back(NAN);
    output.position.push_back(NAN);
    output.position.push_back(NAN);
    output.position.push_back(NAN);    
    output.position.push_back(destination);
    output.position.push_back(NAN);

    pub_.publish(output);

  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

//void getDestination(float dest)
//{
//  x_estimate = x_dest;
//  y_estimate = y_dest;
//  direction = dir;
//}

bool servo(hebi::Stage::Request  &req,
         hebi::Stage::Response &res)
{
    //counter = 0;

    SubscribeAndPublish SAPObject;

    destination = req.stage_req; 
    //SAPObject.getDestination(req.stage_req);

    ros::Rate loop_rate(100);

    //while (counter <50)    //Servo to the stalk for 50 cycles
    while(std::abs(destination-stage_loc) > 0.01)
    {
        ros::spinOnce();
        loop_rate.sleep();
        //counter++;
    }

    res.stage_resp = req.stage_req;

    return true;
}

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "stage");

  ros::NodeHandle n;

  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

  ros::ServiceServer service = n.advertiseService("stage", servo);

  ros::spin();

  return 0;
}

