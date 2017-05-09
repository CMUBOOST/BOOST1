#include "ros/ros.h"
#include "arm_executive/ArmInit.h"
#include "gripper/GripperActuate.h"
#include "gripper/GripperClose.h"
#include "gripper/GripperOpen.h"
#include "hebi/ArmConfigure.h"
#include "hebi/ArmServo.h"
#include "hebi/ArmReverse.h"
#include "hebi/AlphaCentroid.h"
#include "automatic_exposure/AutomaticExposure.h"
#include <cstdlib>


bool run(arm_executive::ArmInit::Request  &req,
         arm_executive::ArmInit::Response &res)
{

//------------------------Turn on camera------------------------

//(turn this into a service call? Right now it just starts up with the launch file)


    ros::NodeHandle nh6;
    ros::ServiceClient client6 = nh6.serviceClient<automatic_exposure::AutomaticExposure>("automatic_exposure");
    automatic_exposure::AutomaticExposure srv6;

    srv6.request.intensity_target = 30;

    ROS_INFO("Populated the request");
    if (client6.call(srv6))
    {
      ROS_INFO_STREAM("Automatic Exposure Called"); //<< srv.response.arm_config_resp);
    }
    else
    {
      ROS_ERROR("Failed to call service automatic_exposure");
      return 1;
    }


    if (req.arm_init_req != "stow")
    {
          //--------------Move Arm to Left/Right Configuration---------------------
          res.arm_init_resp = req.arm_init_req;
          ROS_INFO_STREAM("request:" << req.arm_init_req);
          ros::NodeHandle nh;

          ROS_INFO("Created a new node handle for arm configuration");

          ros::ServiceClient client = nh.serviceClient<hebi::ArmConfigure>("arm_configure");
          hebi::ArmConfigure srv;

          srv.request.arm_config_req = req.arm_init_req;

          ROS_INFO("Populated the request");
          if (client.call(srv))
          {
            ROS_INFO_STREAM("Response from Hebi Command: " << srv.response.arm_config_resp);
          }
          else
          {
            ROS_ERROR("Failed to call service arm_configure");
            return 1;
          }

          //sleep(3);

          std::cout<<"end of sleep timer" << std::endl;


          //---------------------Make Sure Gripper Is Open----------------------

          ros::NodeHandle nh8;

          ROS_INFO("Created a new node handle for gripper opening");

          ros::ServiceClient client8 = nh8.serviceClient<gripper::GripperOpen>("gripper_open");
          gripper::GripperOpen srv8;

          srv8.request.gripper_open_req = req.arm_init_req;  //this is kindof just a placeholder now. the gripper service shouldn't be dependent on left or right

          ROS_INFO("Populated the gripper actuate request");
          if (client8.call(srv8))
          {
            ROS_INFO_STREAM("Response from Gripper Command: " << srv8.response.gripper_open_resp);
          }
          else
          {
            ROS_ERROR("Failed to call service gripper_actuate");
            return 1;
          }


          //---------------------Start Centroid Detection-------------------------

          ros::NodeHandle nh5;

          ROS_INFO("Created a new node handle for alpha centroid detection");

          ros::ServiceClient client5 = nh5.serviceClient<hebi::AlphaCentroid>("alpha_centroid");
          hebi::AlphaCentroid srv5;

          srv5.request.alpha_centroid_req = req.arm_init_req;  //i.e. I'll send 'left' or 'right' as the request

          ROS_INFO("Populated the request");
          if (client5.call(srv5))
          {
            ROS_INFO_STREAM("Response from Alpha Centroid Command: " << srv5.response.alpha_centroid_resp_x << ", " << srv5.response.alpha_centroid_resp_y << ", " << srv5.response.alpha_centroid_resp_z << ", " << srv5.response.alpha_centroid_num_inliers);
          }
          else
          {
            ROS_ERROR("Failed to call service alpha_centroid");
            return 1;
          }

          if (srv5.response.alpha_centroid_num_inliers < 5)
          {
              ROS_ERROR("NOT ENOUGH CENTROIDS!! SETTING TO INIT!!");

              ROS_INFO("Populated the request");
              if (client.call(srv))
              {
                ROS_INFO_STREAM("Response from Hebi Command: " << srv.response.arm_config_resp);
              }
              else
              {
                ROS_ERROR("Failed to call service arm_configure");
                return 1;
              }



          }
          else
          {
            //----------------Visually Servo Arm to Plant------------------------

              ros::NodeHandle nh2;

              ROS_INFO("Created a new node handle for arm servoing");

              ros::ServiceClient client2 = nh2.serviceClient<hebi::ArmServo>("arm_servo");
              hebi::ArmServo srv2;

              srv2.request.arm_servo_req_jointAngle = req.arm_init_req;  //i.e. I'll send 'left' or 'right' as the request
              srv2.request.arm_servo_req_x = srv5.response.alpha_centroid_resp_x;
              srv2.request.arm_servo_req_y = srv5.response.alpha_centroid_resp_y;
              srv2.request.arm_servo_req_z = srv5.response.alpha_centroid_resp_z;

              ROS_INFO("Populated the request");
              if (client2.call(srv2))
              {
                ROS_INFO_STREAM("Response from Servo Command: " << srv2.response.arm_servo_resp);
              }
              else
              {
                ROS_ERROR("Failed to call service arm_servo");
                return 1;
              }


            //-------------------------Actuate Gripper-----------------------------

              //sleep(2);

              ros::NodeHandle nh4;

              ROS_INFO("Created a new node handle for gripper actuation");

              ros::ServiceClient client4 = nh4.serviceClient<gripper::GripperActuate>("gripper_actuate");            //ArmReverse>("arm_reverse");
              gripper::GripperActuate srv4;   //hebi::ArmReverse srv3;

              srv4.request.gripper_actuate_req = req.arm_init_req;  //this is kindof just a placeholder now. the gripper service shouldn't be dependent on left or right

              ROS_INFO("Populated the gripper actuate request");
              if (client4.call(srv4))
              {
                ROS_INFO_STREAM("Response from Gripper Command: " << srv4.response.gripper_actuate_resp);
              }
              else
              {
                ROS_ERROR("Failed to call service gripper_actuate");
                return 1;
              }

              //sleep(2);


        //----------------Move Arm Back to Initial Position------------------------


          ros::NodeHandle nh3;

          ROS_INFO("Created a new node handle for arm reverse");

          ros::ServiceClient client3 = nh3.serviceClient<hebi::ArmReverse>("arm_reverse");
          hebi::ArmReverse srv3;

          srv3.request.arm_reverse_config_req = req.arm_init_req;  //i.e. I'll send 'left' or 'right' as the request
          srv3.request.arm_reverse_x_est_req = srv5.response.alpha_centroid_resp_x;
          srv3.request.arm_reverse_y_est_req = srv5.response.alpha_centroid_resp_y;

          ROS_INFO("Populated the request");
          if (client3.call(srv3))
          {
            ROS_INFO_STREAM("Response from Servo Reverse Command: " << srv3.response.arm_reverse_resp);
          }
          else
          {
            ROS_ERROR("Failed to call service arm_reverse");
            return 1;
          }
       }
    }

  else
  {
      //--------------------------------Close Gripper-----------------------------

      ros::NodeHandle nh7;

      ROS_INFO("Created a new node handle for gripper actuation");

      ros::ServiceClient client7 = nh7.serviceClient<gripper::GripperClose>("gripper_close");
      gripper::GripperClose srv7;

      srv7.request.gripper_close_req = req.arm_init_req;  //this is kindof just a placeholder now. the gripper service shouldn't be dependent on left or right

      ROS_INFO("Populated the gripper actuate request");
      if (client7.call(srv7))
      {
        ROS_INFO_STREAM("Response from Gripper Command: " << srv7.response.gripper_close_resp);
      }
      else
      {
        ROS_ERROR("Failed to call service gripper_actuate");
        return 1;
      }

      sleep(1);

      //--------------Move Arm to Stow Configuration---------------------
      res.arm_init_resp = req.arm_init_req;
      ROS_INFO_STREAM("request:" << req.arm_init_req);
      ros::NodeHandle nh;

      ROS_INFO("Created a new node handle for arm configuration");

      ros::ServiceClient client = nh.serviceClient<hebi::ArmConfigure>("arm_configure");
      hebi::ArmConfigure srv;

      srv.request.arm_config_req = req.arm_init_req;

      ROS_INFO("Populated the request");
      if (client.call(srv))
      {
        ROS_INFO_STREAM("Response from Hebi Command: " << srv.response.arm_config_resp);
      }
      else
      {
        ROS_ERROR("Failed to call service arm_configure");
        return 1;
      }

      //sleep(3);

      std::cout<<"end of sleep timer" << std::endl;
  }


  ROS_INFO_STREAM("sending back response:" << res.arm_init_resp);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_init_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("arm_init", run);
  ROS_INFO("Ready to init the arm");
  ros::spin();

  return 0;
}
