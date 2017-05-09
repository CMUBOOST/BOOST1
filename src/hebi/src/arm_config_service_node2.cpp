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

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

int count = 0;
int globalFlag=0;
int outOfStow = 0;
int leftCompleteFlag= 0;
int firstFunctionCall = 0;
int stowToLeft = 0;

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

        float joint1_leftconfig = -2.3;
        float joint1_rightconfig = 2.3;
        float joint2_leftconfig = 0.5;
        float joint2_rightconfig = -.05;

        //ROS_INFO_STREAM("JOINT 1 POSITION IS: " << msg->position[0]);
        //ROS_INFO_STREAM("JOINT 2 POSITION IS: " << msg->position[2]);

        if (direction == "left")
        {
            leftFunction(msg, joint1_leftconfig, joint2_leftconfig);
        }

        if (direction == "right")
        {
            rightFunction(msg);
        }

        if (direction == "stow")
        {
            ROS_INFO_STREAM("STOWING!");
            stowFunction(msg);
        }

    }

void leftFunction(const sensor_msgs::JointState::ConstPtr& msg, float joint1_leftconfig, float joint2_leftconfig)
{
    sensor_msgs::JointState jointActuate;
    jointActuate.header.stamp = ros::Time::now();
    jointActuate.header.frame_id = "";

    if (firstFunctionCall < 5)
    {
        ROS_INFO("TELLING IT TO IDLE!!");
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(1.0);
        jointActuate.position.push_back(NAN);
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(0);
        pub_.publish(jointActuate);
        firstFunctionCall++;
        return;
    }

    if (msg->position[2] > 1.0)  //this is designed to move link2 from the "right-side" configuration
    {
      ROS_INFO("IF 1");
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(-10.0);
      pub_.publish(jointActuate);
    }

    else if (((msg->position[2] >= -1.0) || (msg->position[0] <= 0.5)) && (msg->position[2] > -2.0)) //the check at -2.0 will prevent activation when moving out of stow because of the angle of motor21
    {
      ROS_INFO("IF 2");
      jointActuate.velocity.push_back(5); //10.0*(msg->position[0]) + 0.1);
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(-5); //10.0*(msg->position[2]) - 0.1);
      pub_.publish(jointActuate);
    }

    else if ((msg->position[2] >= -2.3) && (stowToLeft == 0))
    {
      ROS_INFO("IF 3");
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(2.0*msg->position[2] - 0.1);
      pub_.publish(jointActuate);
    }

    else if ((msg->position[2] > -2.8) && (stowToLeft == 0) && (msg->position[0] <= 0.5))  //This was added to catch a far left configuration. Geez this is getting ugly!!
    {
      ROS_INFO("IF NEW!!");
      jointActuate.velocity.push_back(5);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      pub_.publish(jointActuate);
    }

    else if ((msg->position[2] < -2.3) && (stowToLeft == 0))  //This was added to catch a far left configuration. Geez this is getting ugly!!
    {
      ROS_INFO("IF NEW2!!");
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(5);
      pub_.publish(jointActuate);
      stowToLeft = 1;
    }

    //THESE TWO ELSE-IFS ARE TO BRING IT OUT OF STOW!
    else if ((msg->position[2] <= -2.8) && (msg->position[0] < 0.5))
    {
      ROS_INFO("IF 4");
      jointActuate.velocity.push_back(5);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      pub_.publish(jointActuate);
    }

    else if ((msg->position[2] <= -2.3) && (stowToLeft == 1))
    {
      ROS_INFO("IF 5");
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(5);
      pub_.publish(jointActuate);
    }

    else
    {
        ROS_INFO("TELLING IT TO STOP!!");
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(1.0);
        jointActuate.position.push_back(NAN);
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(0);
        pub_.publish(jointActuate);
        globalFlag = 1;
        leftCompleteFlag = 1;
    }
}

void rightFunction(const sensor_msgs::JointState::ConstPtr& msg)
{
    sensor_msgs::JointState jointActuate;
    jointActuate.header.stamp = ros::Time::now();
    jointActuate.header.frame_id = "";

    //ROS_INFO_STREAM("count is: " << count);

    if (firstFunctionCall < 5)
    {
        ROS_INFO("TELLING IT TO IDLE!!");
        jointActuate.position.push_back(NAN);
        jointActuate.position.push_back(1.0);
        jointActuate.position.push_back(NAN);
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(0);
        pub_.publish(jointActuate);
        firstFunctionCall++;
        return;
    }

    if ((msg->position[0] < 0.5) && (outOfStow == 0) && (msg->position[2] < 0))
    {
      //ROS_INFO("In the first if");
      jointActuate.velocity.push_back(5);
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(0);
      pub_.publish(jointActuate);
    }

      else if (msg->position[2] < -1.0)
      {
        outOfStow = 1;
        //ROS_INFO("In the first if");
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(10.0);
        pub_.publish(jointActuate);
      }

      else if ((msg->position[2] <= 1.0) || (msg->position[0] >= -0.5))
      {
        //ROS_INFO("IN THE SECOND IF");
        jointActuate.velocity.push_back(-5); //10.0*(-0.5-msg->position[0]) - 0.1);
        jointActuate.velocity.push_back(0);
        jointActuate.velocity.push_back(5); //10.0*(1.0-msg->position[2]) + 0.1); //the 1.0 here represents the destination, so destination-current position gives you a proportional gain on speed
        //ROS_INFO_STREAM("Command: " << (10.0*(1.0-msg->position[2]) + 0.1));
        pub_.publish(jointActuate);
      }

      else if (msg->position[2] <= 2.3)
      {
        jointActuate.velocity.push_back(0.0);
        jointActuate.velocity.push_back(0.0);
        jointActuate.velocity.push_back(5);
        pub_.publish(jointActuate);
      }

      else
      {
          ROS_INFO("TELLING IT TO STOP!!");
          jointActuate.position.push_back(NAN);
          jointActuate.position.push_back(1.0);
          jointActuate.position.push_back(NAN);
          jointActuate.velocity.push_back(0);
          jointActuate.velocity.push_back(0);
          jointActuate.velocity.push_back(0);
          pub_.publish(jointActuate);
          globalFlag = 1;
      }
}

void stowFunction(const sensor_msgs::JointState::ConstPtr& msg)
{
    sensor_msgs::JointState jointActuate;
    jointActuate.header.stamp = ros::Time::now();
    jointActuate.header.frame_id = "";

    stowToLeft = 1;

    //if (leftCompleteFlag == 0)
    //{
    //    leftFunction(msg);
    //    globalFlag = 0;
    //}

    if (msg->position[2] > 1.0)
    {
      ROS_INFO("IF 1");
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(-10.0);
      pub_.publish(jointActuate);
    }

    //else if ((msg->position[2] >= -1.0) || (msg->position[0] <= 0.5)) //at stow time, the position of joint 21 goes back to being less than 0.5
    else if (((msg->position[2] >= -1.0) || (msg->position[0] <= 0.5)) && (msg->position[2] > -2.0))
    {
      ROS_INFO("IF 2");
      jointActuate.velocity.push_back(5);//10.0*(msg->position[0]) + 0.1);
      jointActuate.velocity.push_back(0);
      jointActuate.velocity.push_back(-5); //10.0*(msg->position[2]) - 0.1);
      pub_.publish(jointActuate);
    }

    else if (msg->position[2] >= -2.3)
    {
      ROS_INFO("IF 3");
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(2.0*msg->position[2] - 0.1);
      pub_.publish(jointActuate);
    }

    //end of additions

    else if (msg->position[2] >= -3.0)
    {
      ROS_INFO("IF 4");
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(2.0*msg->position[2] - 0.1);
      pub_.publish(jointActuate);
    }

    else if (msg->position[0] > 0)
    {
      jointActuate.velocity.push_back(-5);
      jointActuate.velocity.push_back(0.0);
      jointActuate.velocity.push_back(0.0);
      pub_.publish(jointActuate);
    }

    else
    {
        ROS_INFO("HOLDING POSITION");
        jointActuate.position.push_back(0.0);
        jointActuate.position.push_back(1.0);
        jointActuate.position.push_back(-3.1);
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
  outOfStow = 0;
  leftCompleteFlag = 0;
  firstFunctionCall = 0;
  stowToLeft = 0;


  ROS_INFO_STREAM("sending back response: " << res.arm_config_resp);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");  //rename this but check!!
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("arm_configure", config);  //change this function name!!
  ROS_INFO("Ready to configure the arm.");
  ros::spin();

  return 0;
}
