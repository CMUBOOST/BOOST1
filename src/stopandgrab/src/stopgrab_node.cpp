#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <arm_executive/ArmInit.h>
#include <log4cxx/logger.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace std;

vector<float> waypoints_x; //(define waypoints as a global)
vector<float> waypoints_y;
float tolerance = 0.2; //tolerance in meters
float speed = 1.0; //1 m/s?
float safety = 20.0; //safety factor
float utm_test_x = 9.56;
float utm_test_y = 9.33;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		pub_ = n_.advertise<std_msgs::Bool>("/nav_lock", 1);
		sub_ = n_.subscribe("/utm", 1, &SubscribeAndPublish::callback, this);
	}

    void callback(const std_msgs::Bool::ConstPtr& msg)  //CHANGE THIS SO IT IS A ODOM MESSAGE
	{		

        //---------------------PARSE THE ODOM MESSAGE HERE----------------------
        //(parse UTM)



        //declare message to be published
        std_msgs::Bool nav_lock;

        //----------------------calculate L2 norm---------------
        for(int a=0; a<waypoints_x.size(); a++)
        {
            if((sqrt(pow((utm_test_x - waypoints_x[a]),2)+pow(utm_test_y - waypoints_y[a],2))) < tolerance)  //UTM_TEST IS A PLACEHOLDER FOR REAL UTM
            {
                //-----------------publish nav lock 10 times---------------
                nav_lock.data = 1;
                for(int i=0; i<10; i++)
                {
                    pub_.publish(nav_lock);
                }

                //-------------------perform service call--------------------
                ros::NodeHandle nh;
                ros::ServiceClient client = nh.serviceClient<arm_executive::ArmInit>("arm_init");
                arm_executive::ArmInit srv;

                srv.request.arm_init_req = "left";

                ROS_INFO("Populated the request");
                if (client.call(srv))
                {
                  ROS_INFO_STREAM("Arm Init Left Called");
                }
                else
                {
                  ROS_ERROR("Failed to call service arm_init left");
                }

                srv.request.arm_init_req = "right";

                if (client.call(srv))
                {
                  ROS_INFO_STREAM("Arm Init Right Called");
                }
                else
                {
                  ROS_ERROR("Failed to call service arm_init right");
                }

                srv.request.arm_init_req = "stow";

                if (client.call(srv))
                {
                  ROS_INFO_STREAM("Arm Init Stow Called");
                }
                else
                {
                  ROS_ERROR("Failed to call service arm_init stow");
                }



                //-------------publish nav lock 10 times---------------
                nav_lock.data = 0;
                for(int i=0; i<10; i++)
                {
                    pub_.publish(nav_lock);
                }

                //-------------wait while robot exits L2 zone-------------
                sleep(2*(tolerance/speed)*safety);

            }
        }
	}


private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};





//---------------------------------MAIN-----------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stopgrab_node");
  ros::NodeHandle n;
  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  SubscribeAndPublish SAPObject;



  //-----------------populate waypoints from a txt file (yeah yeah yeah this is ugly)-----------------
  string line;
  ifstream myfile ("/home/boost-1/test3/src/stopandgrab/waypoints/points.txt");
  int counter = 0;
  if (myfile.is_open())
  {
    while (getline (myfile,line))
    {
      istringstream ss(line);
      string token;
      float result;

      while(std::getline(ss, token, ' '))
      {
          result = atof(token.c_str());
          if(counter == 0)
          {
              waypoints_x.push_back(result);
          }
          else if(counter == 1)
          {
              waypoints_y.push_back(result);
          }
          counter++;
      }
      counter = 0;
    }
    myfile.close();
  }
  else cout << "Unable to open file";



  //-----------------run the callback infinitely-------------------
  while(ros::ok())
  {
      ros::spin();
  }

  return 0;
}
