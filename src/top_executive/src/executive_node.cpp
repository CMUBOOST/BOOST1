#include "ros/ros.h"
#include "arm_executive/ArmInit.h"
#include "top_executive/Exec.h"
#include <cstdlib>


bool run(top_executive::Exec::Request  &req,
         top_executive::Exec::Response &res)
{

    //---------------------------LEFT---------------------------

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<arm_executive::ArmInit>("arm_init");
    arm_executive::ArmInit srv;

    srv.request.arm_init_req = "left";

    ROS_INFO("Populated the request");
    if (client.call(srv))
    {
      ROS_INFO_STREAM("Arm Executive Called"); //<< srv.response.arm_config_resp);
    }
    else
    {
      ROS_ERROR("Failed to call service arm_executive");
      return 1;
    }


    //-----------------------------RIGHT----------------------------

    ros::NodeHandle nh2;
    ros::ServiceClient client2 = nh2.serviceClient<arm_executive::ArmInit>("arm_init");
    arm_executive::ArmInit srv2;

    srv2.request.arm_init_req = "right";

    ROS_INFO("Populated the request");
    if (client2.call(srv2))
    {
      ROS_INFO_STREAM("Arm Executive Called"); //<< srv.response.arm_config_resp);
    }
    else
    {
      ROS_ERROR("Failed to call service arm_executive");
      return 1;
    }


    //-----------------------------STOW-------------------------------

    ros::NodeHandle nh3;
    ros::ServiceClient client3 = nh3.serviceClient<arm_executive::ArmInit>("arm_init");
    arm_executive::ArmInit srv3;

    srv3.request.arm_init_req = "stow";

    ROS_INFO("Populated the request");
    if (client3.call(srv3))
    {
      ROS_INFO_STREAM("Arm Executive Called"); //<< srv.response.arm_config_resp);
    }
    else
    {
      ROS_ERROR("Failed to call service arm_executive");
      return 1;
    }

  res.exec_resp = req.exec_req;

  ROS_INFO_STREAM("sending back response:" << res.exec_resp);
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "exec_server"); //used to be arm_init_server
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("exec", run);  //used to be arm_init
  ROS_INFO("Ready to execute");
  ros::spin();

  return 0;
}
