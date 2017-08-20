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

#include "servoVars.hpp"

using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

std::string direction;

int count = 0;
float x_estimate;
float y_estimate;

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
        /*
        float j2_Lconfig = -2.08;
        float j2_Rconfig = 2.08;
        float j1_Lconfig = 0.5;
        float j1_Rconfig = -0.5;
        */

		  jointActuate.header.stamp = ros::Time::now();
		  jointActuate.header.frame_id = "";	

          if ((direction == "left") && (msg->position[0] < j1_Lconfig) && (count<400))
		  {
            ROS_DEBUG_STREAM("Count is: " << count);

            float gain= float(count)/500;
            float mPError1 = j1_Lconfig-msg->position[0];
            float command1 = msg->position[0]+gain*mPError1;

            jointActuate.position.push_back(command1);
            jointActuate.position.push_back(NAN);

            //-------------------Calculate desired position of joint 2---------------
            float hyp = 0.3;
            float joint2_x = hyp*cos(command1);
            float joint2_y = hyp*sin(command1);
            //ROS_DEBUG_STREAM("Joint2_x: " << joint2_x << ", Joint2_y: " << joint2_y);

            //Calculate camera position (I could subscribe to the topic but this is easier)
            float hyp_camera = .245;
            float camera_x = hyp_camera*cos(command1 + msg->position[2]) + joint2_x;
            float camera_y = hyp_camera*sin(command1 + msg->position[2]) + joint2_y;
            //ROS_DEBUG_STREAM("Camera_x: " << camera_x << ", Camera_y: " << camera_y);

            float line1 = atan2((joint2_y-camera_y),(joint2_x-camera_x));
            float line2 = atan2((joint2_y-y_estimate),(joint2_x-x_estimate));
            float line_diff = line1-line2;
            //ROS_DEBUG_STREAM("Line diff: " << line_diff);
            //-----------------------------------------------------------------------

            jointActuate.position.push_back(msg->position[2] - line_diff);
            pub_.publish(jointActuate);
            ROS_DEBUG_STREAM("In the left IF statement: "  << (msg->position[2] - line_diff));

		  }  
          else if ((direction == "left") && (count<480))
		  {
            //ROS_DEBUG_STREAM("IF2, J1 POSITION: " << msg->position[0]);
            float gain = float(count-400)/80.0;
            float mPError2 = abs(j2_Lconfig-msg->position[2]);
            float command2 = msg->position[2]+gain*mPError2;
            jointActuate.position.push_back(j1_Lconfig);
            jointActuate.position.push_back(NAN);
            jointActuate.position.push_back(command2);
            pub_.publish(jointActuate);
            ROS_DEBUG_STREAM("In the left ELSEIF statement" << j2_Lconfig << ", " << msg->position[2] << ", " << mPError2 << ", " << command2);
		  }  

          //----this elseif didn't exist before 7/28
          else if ((direction == "left") && (count>=480))
          {
              //ROS_DEBUG("LOCKING THE ARM");
              jointActuate.velocity.push_back(0);
              jointActuate.velocity.push_back(0);
              jointActuate.velocity.push_back(0);
              jointActuate.position.push_back(j1_Lconfig);
              jointActuate.position.push_back(NAN);
              jointActuate.position.push_back(j2_Lconfig);
              pub_.publish(jointActuate);
              //ROS_DEBUG("In the left FORCE statement");
          }

          else if ((direction == "right") && (msg->position[0] > j1_Rconfig) && (count<400))
		  {
            ROS_DEBUG_STREAM("Count is: " << count);

            float gain= float(count)/500;
            float mPError1 = j1_Rconfig-msg->position[0];
            float command1 = msg->position[0]+gain*mPError1;

            jointActuate.position.push_back(command1);
            jointActuate.position.push_back(NAN);

            //-------------------Calculate desired position of joint 2---------------
            float hyp = 0.3;
            float joint2_x = hyp*cos(command1);
            float joint2_y = hyp*sin(command1);
            ROS_DEBUG_STREAM("Joint2_x: " << joint2_x << ", Joint2_y: " << joint2_y);

            //Calculate camera position (I could subscribe to the topic but this is easier)
            float hyp_camera = .245;
            float camera_x = hyp_camera*cos(command1 + msg->position[2]) + joint2_x;
            float camera_y = hyp_camera*sin(command1 + msg->position[2]) + joint2_y;
            ROS_DEBUG_STREAM("Camera_x: " << camera_x << ", Camera_y: " << camera_y);

            float line1 = atan2((joint2_y-camera_y),(joint2_x-camera_x));
            float line2 = atan2((joint2_y-y_estimate),(joint2_x-x_estimate));
            float line_diff = line1-line2;
            ROS_DEBUG_STREAM("Line diff: " << line_diff);
            //-----------------------------------------------------------------------

            jointActuate.position.push_back(msg->position[2] - line_diff);
            pub_.publish(jointActuate);

		  } 

          else if ((direction == "right") && (count<480))
		  {
              ROS_DEBUG_STREAM("IF2, J1 POSITION: " << msg->position[0]);
              float gain = float(count-400)/80.0;
              float mPError2 = abs(j2_Rconfig-msg->position[2]);
              float command2 = msg->position[2]+gain*mPError2;
              jointActuate.position.push_back(j1_Rconfig);
              jointActuate.position.push_back(NAN);
              jointActuate.position.push_back(command2);
              pub_.publish(jointActuate);
		  }  

          //this elseif had all NANs for position before 7/28
          else if ((direction == "right") && (count>=480))
          {
              ROS_DEBUG("LOCKING THE ARM");
              jointActuate.velocity.push_back(0);
              jointActuate.velocity.push_back(0);
              jointActuate.velocity.push_back(0);
              jointActuate.position.push_back(j1_Rconfig);
              jointActuate.position.push_back(NAN);
              jointActuate.position.push_back(j2_Rconfig);
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

  x_estimate = req.arm_reverse_x_est_req;
  y_estimate = req.arm_reverse_y_est_req;
  res.arm_reverse_resp = req.arm_reverse_config_req;
  direction = req.arm_reverse_config_req;

  ROS_INFO_STREAM("Reverse direction request: " << req.arm_reverse_config_req);

  SubscribeAndPublish SAPObject;

  //--------------Perform FK to Estimate Location of

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
  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

  ros::ServiceServer service = n.advertiseService("arm_reverse", reverse);
  ROS_INFO("Ready to reverse the arm!");
  ros::spin();

  return 0;
}
