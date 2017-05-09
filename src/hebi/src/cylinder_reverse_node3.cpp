#include "ros/ros.h"
#include "hebi/ArmReverse.h"
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
#include <log4cxx/logger.h>

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

std::string direction;

int count = 0;

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

		  if ((direction == "left") && (msg->position[0] < 0.5))
		  {
            ROS_DEBUG_STREAM("JOINT 1 POSITION IS LESS THAN 0.5: " << msg->position[0]);
            ROS_DEBUG_STREAM("JOINT 2 POSITION IS: " << msg->position[2]);
            jointActuate.velocity.push_back(0.5-msg->position[0]+0.3);
            jointActuate.velocity.push_back(0);
            jointActuate.velocity.push_back(-2.3-msg->position[2]-0.3);
            //jointActuate.velocity.push_back(.3*M_PI);
            //jointActuate.velocity.push_back(0);
            //jointActuate.velocity.push_back(-.2*(M_PI+msg->position[2]));
			pub_.publish(jointActuate);
		  }  
          else if ((direction == "left") && ((msg->position[0] >= 0.5)||(msg->position[2] > -2.3)))
		  {
            ROS_DEBUG_STREAM("JOINT 1 POSITION IS GREATER THAN 0.5: " << msg->position[0]);
		  	jointActuate.velocity.push_back(0);
		  	jointActuate.velocity.push_back(0);
		  	jointActuate.velocity.push_back(0);
            jointActuate.position.push_back(0.5);
            jointActuate.position.push_back(1);
            jointActuate.position.push_back(-2.3);
			pub_.publish(jointActuate);
		  }  
          else if ((direction == "right") && ((msg->position[0] > -0.5)||(msg->position[2] < 2.3)))
		  {

            ROS_DEBUG_STREAM("JOINT 1 POSITION IS GREATER THAN -0.5: " << msg->position[0]);
            ROS_DEBUG_STREAM("JOINT 2 POSITION IS: " << msg->position[2]);
            jointActuate.velocity.push_back(-0.5-msg->position[0]-0.3);
		  	jointActuate.velocity.push_back(0);
            jointActuate.velocity.push_back(2.3-msg->position[2]+0.3);//.2*(M_PI+msg->position[2]));
			pub_.publish(jointActuate);
		  } 

		  else if ((direction == "right") && (msg->position[0] <= -0.5))
		  {
            ROS_DEBUG_STREAM("JOINT 1 POSITION IS LESS THAN -0.5: " << msg->position[0]);
		  	jointActuate.velocity.push_back(0);
		  	jointActuate.velocity.push_back(0);
		  	jointActuate.velocity.push_back(0);
            jointActuate.position.push_back(-0.5);
            jointActuate.position.push_back(1);
            jointActuate.position.push_back(2.3);
			pub_.publish(jointActuate);
		  }  
		  else
		  {
			ROS_WARN("WRONG COMMAND!!");
		  }
		

	}


private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

bool reverse(hebi::ArmReverse::Request  &req,
         hebi::ArmReverse::Response &res)
{

  res.arm_reverse_resp = req.arm_reverse_req;
  direction = req.arm_reverse_req;  

  ROS_INFO_STREAM("Reverse direction request: " << req.arm_reverse_req);

  SubscribeAndPublish SAPObject;

  ros::Rate loop_rate(100);
  count = 0;

  while(count<500)
  {
	  ros::spinOnce();
	  loop_rate.sleep();
	  ++count;	
  }

  ROS_INFO_STREAM("sending back response: " << res.arm_reverse_resp);

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "reverse_server");
  ros::NodeHandle n;
  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  ros::ServiceServer service = n.advertiseService("arm_reverse", reverse);
  ROS_INFO("Ready to reverse the arm!");
  ros::spin();

  return 0;
}
