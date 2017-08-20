#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/NavSatFix.h"
#include <iostream>
#include <fstream>
//using namespace std;

float prior = 0;
float prioraccepted = 0;
int flag = 0;
int gps_flag = 0;
float max = 0;
int counter = 0;

void gps_Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    if(gps_flag = 1){
        std::ofstream myfile;
        myfile.open("/home/boost-4/boost4_ws/src/data_recording/data/datalog.txt", std::ios::app);
        myfile << gps_msg->latitude;
        myfile << "latitude ";
        myfile << gps_msg->longitude;
        myfile << "longitude";
        myfile.close();
        gps_flag = 0;
    }
}

void load_Callback(const std_msgs::Float32::ConstPtr& msg)
{	
    ros::NodeHandle nh;
    //std::cout << msg->data << std::endl;

    if(flag ==1){
        if(msg->data > max){
            max = msg->data;
        }

        counter++;
        if(counter>4000){
            ROS_INFO_STREAM("max reading: " << max);
            std::ofstream myfile;
            myfile.open("/home/boost-4/boost4_ws/src/data_recording/data/datalog.txt", std::ios::app);
            myfile << "\n";
            myfile << max;
            myfile << " lbs, ";
            myfile.close();
            flag = 0;
            max = 0;
            counter = 0;
            gps_flag = 1;
        }
    }
}

void start_Callback(const std_msgs::Int16::ConstPtr& start_msg)
{
    //ROS_INFO_STREAM("Start message: " << start_msg->data);
    flag = 1;
}

    /*
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
    ros::Subscriber dia_sub = nh.subscribe("stalk_dia", 1000, diaCallback);
    */



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

  ros::Subscriber load_sub = n.subscribe("load_cell_pounds", 1000, load_Callback);
  ros::Subscriber start_sub = n.subscribe("start", 1000, start_Callback);
  ros::Subscriber gps_sub = n.subscribe("fix", 1000, gps_Callback);    
  //ros::Subscriber presence_sub = n.subscribe("stalk_presence", 1000, presenceCallback);

  ros::spin();

  return 0;
}
