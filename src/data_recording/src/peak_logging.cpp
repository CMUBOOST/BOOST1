#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include <fstream>
//using namespace std;

float max;
float prior = 0;
float prioraccepted = 0;

void Callback(const std_msgs::Float32::ConstPtr& msg)
{

    if((msg->data)>5.0 && prior>5.0)
    {
        //ROS_INFO_STREAM("I heard: " << msg->data);

        if((msg->data)>prioraccepted)
        {
            max = msg->data;
            prioraccepted = max;
        }
        //counter ++;
    }
    else if(max > 5.0)
    {
        ROS_INFO_STREAM("Maximum at load cell is: " << max);
        std::ofstream myfile;
        myfile.open("/home/boost-1/test3/src/data_recording/data/datalog.txt", std::ios::app);
        myfile << "\n Writing this to a file: ";
        myfile << max;
        myfile << "lbs, ";
        myfile.close();
        max = 5.0;
        prioraccepted = 0;
    }
    else
    {
        prioraccepted = msg->data;
    }

    prior = msg->data;
}

void diaCallback(const std_msgs::Int32::ConstPtr& msg)
{
    std::ofstream myfile;
    myfile.open("/home/boost-1/test3/src/data_recording/data/datalog.txt", std::ios::app);
    myfile << msg->data;
    myfile << "mm dia ";
    myfile.close();
}

/*
void presenceCallback(const std_msgs::Int16::ConstPtr& msg)
{
    std::ofstream myfile;
    myfile.open("/home/boost-1/test3/src/data_recording/data/datalog.txt", std::ios::app);
    //myfile << msg->data;
    myfile << "\n no stalk!!, ";
    myfile.close();
}
*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("load_cell_pounds", 1000, Callback);
  ros::Subscriber dia_sub = n.subscribe("stalk_dia", 1000, diaCallback);
  //ros::Subscriber presence_sub = n.subscribe("stalk_presence", 1000, presenceCallback);

  ros::spin();

  return 0;
}
