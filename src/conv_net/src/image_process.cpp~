#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "conv_net/ImageProcess.h"
#include <sstream>
#include <math.h>

float destination;
int counter;
float stage_loc;
int startFlag;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //pub_ = n_.advertise<sensor_msgs::JointState>("/joint_commands", 1);
    //sub_ = n_.subscribe("/joint_states", 1, &SubscribeAndPublish::callback, this);
  }

  /*
  void callback(const sensor_msgs::JointState::ConstPtr& jointStatePtr)
  {
    if(startFlag == 1 && std::abs(destination-stage_loc) > 0.01){
	    stage_loc = jointStatePtr->position[5];
	    //std::cout << "In the STAGE CALLBACK" << std::endl;

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
    else if(startFlag == 1){
        startFlag = 0;    
    }

  }
  */

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

bool detect(conv_net::ImageProcess::Request &req, conv_net::ImageProcess::Response &res)
{
    
    ROS_INFO("IN THE SERVICE CALLBACK");
    startFlag = 1;
    //destination = req.stage_req; 

    /*
    ros::Rate loop_rate(100);

    while(std::abs(destination-stage_loc) > 0.01)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    */

    res.image_process_resp = req.image_process_req;

    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_process");

  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("image_process", detect);

  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
