#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <math.h>
#include <unistd.h>
#include <log4cxx/logger.h>

#include "servoVars.hpp"

using namespace sensor_msgs;

int globalFlag=0;
int approachingStow = 0;

std::string direction;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        pub_ = n_.advertise<sensor_msgs::JointState>("joint_commands", 1);
        sub_ = n_.subscribe("/joint_states", 1, &SubscribeAndPublish::callback, this);
    }

    void callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        ROS_INFO("In the callback");
        sensor_msgs::JointState jointActuate;

        //do stuff
        jointActuate.header.stamp = ros::Time::now();
        jointActuate.header.frame_id = "";
        stowFunction(msg, j1_Lconfig, j2_Lconfig);
    }

    void configFunction(float position_array[], float velocity_array[])  {

        sensor_msgs::JointState jointActuate;
        jointActuate.header.stamp = ros::Time::now();
        jointActuate.header.frame_id = "";

        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(position_array [0]);
        jointActuate.position.push_back(position_array [1]);
        jointActuate.position.push_back(position_array [2]);
        jointActuate.velocity.push_back(NAN);
        jointActuate.velocity.push_back(NAN);
        jointActuate.velocity.push_back(NAN);
        jointActuate.velocity.push_back(NAN);
        jointActuate.velocity.push_back(velocity_array [0]);
        jointActuate.velocity.push_back(velocity_array [1]);
        jointActuate.velocity.push_back(velocity_array [2]);
        pub_.publish(jointActuate);
    }  

    void stowFunction(const sensor_msgs::JointState::ConstPtr& msg, float j1_Lconfig, float j2_Lconfig)
    {

        //this is designed to move link2 from the "right-side" configuration
        if (msg->position[6] > 1.0)
        {
            ROS_DEBUG("IF 1, STOW");
            float position_array [3] = { NAN, NAN, NAN };
            float velocity_array [3] = { 0, 0, -10 };
            configFunction(position_array, velocity_array); 
        }

        //This is the double move. Changed on 7/24 to be an && instead of ||
        else if (((msg->position[6] >= -1.0) && (msg->position[4] <= j1_Lconfig)) && (msg->position[6] > -2.0))
        {
          ROS_DEBUG("IF 2, STOW");
            float position_array [3] = { NAN, NAN, NAN };
            float velocity_array [3] = { 5, 0, -5 };
            configFunction(position_array, velocity_array); 
        }


        //-------------------------Added 7/24----------------------
        //This is a catch in case the first link is closer to the center
        else if ((msg->position[4] <= j1_Lconfig) && (approachingStow == 0))
        {
          ROS_DEBUG_STREAM("IF 3 Joint1, STOW: " << msg->position[4]);
            float position_array [3] = { NAN, NAN, NAN };
            float velocity_array [3] = { 5, 0, 0 };
            configFunction(position_array, velocity_array); 
        }
        //---------------------------------------------------------


        //When going from left to right, joint 1 hits its limit first, so this moves joint 2
        else if (msg->position[6] >= j2_Lconfig)
        {
          ROS_DEBUG_STREAM("IF 3 Joint2, STOW: " << msg->position[6]);
            float position_array [3] = { NAN, NAN, NAN };
            float velocity_array [3] = { 0, 0, -5 };
            configFunction(position_array, velocity_array); 
        }


        else if (msg->position[6] >= -3.0)
        {
          ROS_DEBUG_STREAM("IF 4, STOW");
            float position_array [3] = { NAN, NAN, NAN };
            float velocity_array [3] = { 0, 0, -5 };
            configFunction(position_array, velocity_array); 
          approachingStow = 1;
        }

        else if ((msg->position[4] > 0) && (approachingStow == 1))
        {
          ROS_DEBUG("IF 5, STOW");
            float position_array [3] = { NAN, NAN, NAN };
            float velocity_array [3] = { -5, 0, 0 };
            configFunction(position_array, velocity_array); 
        }

        else
        {
            ROS_INFO("HOLDING POSITION");
            float position_array [3] = { 0.0, NAN, -3.14 };
            float velocity_array [3] = { 0, 0, 0 };
            configFunction(position_array, velocity_array); 
            globalFlag = 1;
        }
    }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_stow_initial");
  ros::NodeHandle n;
  SubscribeAndPublish SAPObject;
  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  ROS_INFO("Ready to configure the arm.");
  while(ros::ok() && globalFlag==0){
    ros::spinOnce();
  }

  return 0; 
}