#include "ros/ros.h"
#include "hebi/AddTwoInts.h"
#include "hebi/ArmConfigure.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <string>
#include <math.h>
#include <unistd.h>
#include <log4cxx/logger.h>

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

#include "servoVars.hpp"

using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

int count = 0;
int globalFlag=0;
int comingOutOfStow = 0;
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
        sensor_msgs::JointState jointActuate;

        //do stuff
        jointActuate.header.stamp = ros::Time::now();
        jointActuate.header.frame_id = "";

        /*
        float j2_Lconfig = -2.08;
        float j2_Rconfig = 2.08;
        float j1_Lconfig = 0.5;
        float j1_Rconfig = -0.5;
        */

        if (direction == "left")
        {
            leftFunction(msg, j1_Lconfig, j2_Lconfig);
        }

        if (direction == "right")
        {
            rightFunction(msg, j1_Rconfig, j2_Rconfig);
        }

        if (direction == "stow")
        {
            //ROS_DEBUG("STOWING!");
            stowFunction(msg, j1_Lconfig, j2_Lconfig);
        }

    }

void leftFunction(const sensor_msgs::JointState::ConstPtr& msg, float j1_Lconfig, float j2_Lconfig)
{
    sensor_msgs::JointState jointActuate;
    jointActuate.header.stamp = ros::Time::now();
    jointActuate.header.frame_id = "";

    //Catch if the arm is coming out of stow
    if ((msg->position[2]) < -2.6)
        comingOutOfStow = 1;

    if (msg->position[2] > 1.0)  //this is designed to move link2 from the "right-side" configuration
    {
      ROS_DEBUG("IF 1, LEFT");
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(-10.0);
      pub_.publish(jointActuate);
    }

    //This is the double move
    else if (((msg->position[2] >= -1.0) && (msg->position[0] <= j1_Lconfig))) //&& (msg->position[2] > -2.0)) //the check at -2.0 will prevent activation when moving out of stow because of the angle of motor21
    {
      ROS_DEBUG_STREAM("IF 2, LEFT: " << msg->position[0]);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(5);
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(-5);
      pub_.publish(jointActuate);
    }

    //This is a catch in case the first link is closer to the center
    else if (msg->position[0] <= j1_Lconfig)
    {
      ROS_DEBUG_STREAM("IF 3 Joint1, LEFT: " << msg->position[0]);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(5);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      pub_.publish(jointActuate);
    }

    //When going from right to left, joint 1 hits its limit, so this moves joint 2
    else if ((msg->position[2] >= j2_Lconfig) && (comingOutOfStow == 0))
    {
      ROS_DEBUG_STREAM("IF 3 Joint2, LEFT: " << msg->position[2]);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(-5);
      pub_.publish(jointActuate);
    }

    //This will move joint 2 when coming out of stow. It might overshoot a little bit
    else if ((msg->position[2] < j2_Lconfig) && (comingOutOfStow == 1))
    {
      ROS_DEBUG_STREAM("IF 5, LEFT: " << msg->position[2]);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(5);
      pub_.publish(jointActuate);
    }

    else
    {
        ROS_DEBUG("TELLING IT TO STOP!!");
        jointActuate.position.push_back(j1_Lconfig);  //changed 7/28
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(j2_Lconfig); //changed 7/28
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(0);
        pub_.publish(jointActuate);
        globalFlag = 1;
    }
}

void rightFunction(const sensor_msgs::JointState::ConstPtr& msg, float j1_Rconfig, float j2_Rconfig)
{
    sensor_msgs::JointState jointActuate;
    jointActuate.header.stamp = ros::Time::now();
    jointActuate.header.frame_id = "";

    //ROS_INFO_STREAM("count is: " << count);

    //@TODO: Here I use comingOutOfStow differently than before. It still works, but I should make the variable treated the same way

    //This rotates link1 as the first move out of stow
    if ((msg->position[0] < 0.5) && (comingOutOfStow == 0) && (msg->position[2] < 0))
    {
      ROS_DEBUG("IF 1, RIGHT");
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(5);
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(0);
      pub_.publish(jointActuate);
    }

      //This is the single move of link 2
      else if (msg->position[2] < -1.0)
      {
        comingOutOfStow = 1;
        ROS_DEBUG("IF 2, RIGHT");
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(NAN);
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(10.0);
        pub_.publish(jointActuate);
      }

      //This is the double move
      else if ((msg->position[2] <= 1.0) && (msg->position[0] >= j1_Rconfig))
      {
        ROS_DEBUG("IF 3, RIGHT");
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(NAN);
        jointActuate.velocity.push_back(-5);
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(5);
        pub_.publish(jointActuate);
      }

    //This is a catch in case the first link is closer to the center
    else if (msg->position[0] >= j1_Rconfig)
    {
      ROS_DEBUG_STREAM("IF 3 Joint1, RIGHT: " << msg->position[0]);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(-5);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      pub_.publish(jointActuate);
    }

    //When going from left to right, joint 1 hits its limit first, so this moves joint 2
      else if (msg->position[2] <= j2_Rconfig)
      {
        ROS_DEBUG_STREAM("IF 3 Joint2, RIGHT: " << msg->position[2]);
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(NAN);
        jointActuate.velocity.push_back(0.0);
        jointActuate.velocity.push_back(0.0);
        jointActuate.velocity.push_back(5);
        pub_.publish(jointActuate);
      }

      else
      {
          ROS_DEBUG("TELLING IT TO STOP!!");
          jointActuate.position.push_back(j1_Rconfig);  //changed 7/28
          jointActuate.position.push_back(NAN);
          jointActuate.position.push_back(j2_Rconfig);  //changed 7/28
          jointActuate.velocity.push_back(0);
          jointActuate.velocity.push_back(0);
          jointActuate.velocity.push_back(0);
          pub_.publish(jointActuate);
          globalFlag = 1;
      }
}

void stowFunction(const sensor_msgs::JointState::ConstPtr& msg, float j1_Lconfig, float j2_Lconfig)
{
    sensor_msgs::JointState jointActuate;
    jointActuate.header.stamp = ros::Time::now();
    jointActuate.header.frame_id = "";

    //this is designed to move link2 from the "right-side" configuration
    if (msg->position[2] > 1.0)
    {
      ROS_DEBUG("IF 1, STOW");
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(-10.0);
      pub_.publish(jointActuate);
    }

    //This is the double move. Changed on 7/24 to be an && instead of ||
    else if (((msg->position[2] >= -1.0) && (msg->position[0] <= j1_Lconfig)) && (msg->position[2] > -2.0))
    {
      ROS_DEBUG("IF 2, STOW");
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(5);
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(-5);
      pub_.publish(jointActuate);
    }


    //-------------------------Added 7/24----------------------
    //This is a catch in case the first link is closer to the center
    else if ((msg->position[0] <= j1_Lconfig) && (approachingStow == 0))
    {
      ROS_DEBUG_STREAM("IF 3 Joint1, STOW: " << msg->position[0]);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(5);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      pub_.publish(jointActuate);
    }
    //---------------------------------------------------------


    //When going from left to right, joint 1 hits its limit first, so this moves joint 2
    else if (msg->position[2] >= j2_Lconfig)
    {
      ROS_DEBUG_STREAM("IF 3 Joint2, STOW: " << msg->position[2]);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(-5);
      pub_.publish(jointActuate);
    }


    else if (msg->position[2] >= -3.0)
    {
      ROS_DEBUG_STREAM("IF 4, STOW");
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(-5);
      pub_.publish(jointActuate);
      approachingStow = 1;
    }

    else if ((msg->position[0] > 0) && (approachingStow == 1))
    {
      ROS_DEBUG("IF 5, STOW");
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.position.push_back(NAN);
      jointActuate.velocity.push_back(-5);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      pub_.publish(jointActuate);
    }


    else
    {
        ROS_INFO("HOLDING POSITION");
        jointActuate.position.push_back(0.0);
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(-3.14);
        jointActuate.velocity.push_back(0.0);
        jointActuate.velocity.push_back(0.0);
        jointActuate.velocity.push_back(0.0);
        pub_.publish(jointActuate);
        globalFlag = 1;
    }

}

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};


bool config(hebi::ArmConfigure::Request  &req,
         hebi::ArmConfigure::Response &res)
{

  res.arm_config_resp = req.arm_config_req;
  direction = req.arm_config_req;

  ROS_INFO_STREAM("request: " << req.arm_config_req);

  SubscribeAndPublish SAPObject;

  ros::Rate loop_rate(100);
  count = 0;

  while(globalFlag == 0)
  {
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
  }

  globalFlag = 0;
  comingOutOfStow = 0;
  approachingStow = 0;

  ROS_INFO_STREAM("sending back response: " << res.arm_config_resp);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");  //rename this but check!!
  ros::NodeHandle n;
  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  ros::ServiceServer service = n.advertiseService("arm_configure", config);  //change this function name!!
  ROS_INFO("Ready to configure the arm.");
  ros::spin();

  return 0;
  
  
  
}
