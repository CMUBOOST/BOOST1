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

hebi::Group* group_g = NULL;

std_msgs::String LastCommand;

bool add(hebi::ArmConfigure::Request  &req,
         hebi::ArmConfigure::Response &res)
{

  res.arm_config_resp = req.arm_config_req;  

  ROS_INFO_STREAM("request: " << req.arm_config_req);
  ROS_INFO_STREAM("sending back response: " << res.arm_config_resp);

  //ros::NodeHandle nh;

  //ros::Subscriber sub = nh.subscribe("/joint_states", 1, myCallback);

  //while (globalcounter < total_steps)
  //{
  //  ros::spinOnce();
  //}

  std::vector<std::string> joint_names = {"X-00021", "X-00022", "X-00023"};
  std::vector<std::string> family_names = {"BOOST", "BOOST", "BOOST"};

  hebi::Lookup lookup;

  // Get the group
  for (int i = 0; i < joint_names.size(); i++)
  {
    std::cout << "looking for: " << std::endl;
    std::cout << joint_names[i] << std::endl;
    std::cout << family_names[i] << std::endl;
  }
  std::unique_ptr<hebi::Group> group(lookup.getGroupFromNames(joint_names, family_names, 1000));
  if (!group)
  {
    ROS_INFO("Could not find modules on network! Quitting!");
    return -1;
  }

  // THIS IS A HACK to get around limited callback options for ROS subscribe call and the lack of a class for this node.
  group_g = group.get();

  std::cout << "Found modules!" << std::endl;
  ROS_INFO("Found modules!");

  hebi::GroupCommand cmd(3);
  ROS_INFO("About to command the actuator");


  if (req.arm_config_req == "left" && req.arm_config_req != LastCommand.data)
  {

      cmd[0].actuatorCommand().setPosition(NAN);
      cmd[1].actuatorCommand().setPosition(1);
      cmd[2].actuatorCommand().setPosition(1.0);
      ROS_INFO("group_g");
      group_g->sendCommand(cmd);
      usleep(500000);


      cmd[0].actuatorCommand().setPosition(0);
      cmd[1].actuatorCommand().setPosition(1);
      cmd[2].actuatorCommand().setPosition(0);
      ROS_INFO("group_g");
      group_g->sendCommand(cmd);
      usleep(500000);

      cmd[0].actuatorCommand().setPosition(0.5);
      cmd[1].actuatorCommand().setPosition(1);
      cmd[2].actuatorCommand().setPosition(-1.0);
      ROS_INFO("group_g");
      group_g->sendCommand(cmd);
      usleep(500000);

      cmd[0].actuatorCommand().setPosition(0.4);
      cmd[1].actuatorCommand().setPosition(1);
      cmd[2].actuatorCommand().setPosition(-2.0);
      ROS_INFO("group_g");
      group_g->sendCommand(cmd);
  }
  else if (req.arm_config_req == "right"  && req.arm_config_req != LastCommand.data)
  {
      cmd[0].actuatorCommand().setPosition(NAN);
      cmd[1].actuatorCommand().setPosition(1);
      cmd[2].actuatorCommand().setPosition(-1.0);
      ROS_INFO("group_g");
      group_g->sendCommand(cmd);
      usleep(500000);

      cmd[0].actuatorCommand().setPosition(0);
      cmd[1].actuatorCommand().setPosition(1);
      cmd[2].actuatorCommand().setPosition(0);
      ROS_INFO("group_g");
      group_g->sendCommand(cmd);
      usleep(500000);

      cmd[0].actuatorCommand().setPosition(-0.5);
      cmd[1].actuatorCommand().setPosition(1);
      cmd[2].actuatorCommand().setPosition(1.0);
      ROS_INFO("group_g");
      group_g->sendCommand(cmd);
      usleep(500000);

      cmd[0].actuatorCommand().setPosition(-0.4);
      cmd[1].actuatorCommand().setPosition(1);
      cmd[2].actuatorCommand().setPosition(2.0);
      ROS_INFO("group_g");
      group_g->sendCommand(cmd);
  }
  else
    ROS_INFO("Incorrect request!!");

  LastCommand.data = req.arm_config_req;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");  //rename this but check!!
  ros::NodeHandle n;

  LastCommand.data = "center";

  ros::ServiceServer service = n.advertiseService("arm_configure", add);  //change this function name!!
  ROS_INFO("Ready to configure the arm.");
  ros::spin();

  return 0;
}
