#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include <cstdio>
#include <string>
#include <sstream>
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "serial/serial.h"
#include <unistd.h>

#define NMEA_LENGTH 33

std::string to_string(int i){
  std::ostringstream os;
  os << i;
  return os.str();
}

serial::Serial syncboard;
ros::Publisher timestamps_pub;

bool handshake(){
  int counter = 0;
  while(ros::ok()){
    syncboard.flush();
    syncboard.write("h");
    std::string result = syncboard.readline();
    if (result.length() > 0){
      ROS_INFO("Connected to syncboard.");
      return true;
    }
    if(counter++ > 50){
      ROS_WARN_ONCE("Connecting to syncboard is taking longer than expected.");
    }
    ros::Rate(10).sleep();
  }
  ROS_WARN("Syncboard handshake failed.");
  return false;  
}

bool read_nmea_string(std::string *return_string){
  while(syncboard.available() > 0){
    try{
      std::string nmea_string = syncboard.readline();
      int start_pose = nmea_string.find("$");
      if (start_pose >= 0){
        nmea_string = nmea_string.substr(start_pose);
        *return_string = nmea_string;
        return true;
      }  
    }
    catch (int excepton){
      ROS_ERROR_STREAM("An exception occurred while reading from serial port:" << excepton);
      *return_string = excepton;
      return false;
    } 
  }
  ROS_ERROR("Error parsing NMEA string.");
  *return_string = "Error parsing NMEA string.";
  return false;
}

void publish_nmea_string(std::string nmea_string){
  std_msgs::String nmea_msg;
  ros::Time computer_stamp = ros::Time::now();
  std::string syncbox_stamp = nmea_string.substr(7,9);
  std::string description = ":(syncbox_seconds computer_seconds computer_nanoseconds)"; 
  std::string msg_str = syncbox_stamp +":"+ to_string(computer_stamp.sec) +":"+ to_string(computer_stamp.nsec) + description; //TODO change this format (requires changing pgr_camera)
  nmea_msg.data = msg_str;
  timestamps_pub.publish(nmea_msg);
}

bool trigger_callback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
  syncboard.write("t");
  std::string nmea_string;
  ros::Time timeout = ros::Time::now() + ros::Duration(1);
  while(ros::Time::now() < timeout){
    if (syncboard.available() > NMEA_LENGTH) {
      if(read_nmea_string(&nmea_string)){
        res.success = true; 
        publish_nmea_string(nmea_string);
        res.message = nmea_string;
        return true;
      }
      else{ 
        res.message = nmea_string;
        res.success = false; 
        return true;
      }
    }
  }
  res.message = "Timed out waiting for NMEA message.";
  res.success = false; 
  return true;
}

bool burst_callback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
  static unsigned int count=0;

  res.success = true; 
  res.message = "Started syncboard burst trigger"; //this trigger defaults to 5hz and is independent of robot position
  geometry_msgs::Twist output;

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/camera_stitch/cmd_vel", 1);

  output.linear.x = 0.5;		
  output.linear.y = 0.0;
  output.linear.z = 0.0;
  output.angular.x = 0.0;
  output.angular.y = 0.0;
  output.angular.z = 0.0;
  pub.publish(output);

  if (count==0) {
    usleep(100000); //adding in a sleep because the transmission delay to move is so long
  	syncboard.write("b");
  }

  for(int i=0; i<5; i++){
    std::string result = syncboard.readline();
	if(i<2){
      output.linear.x = 0.5;		
      output.linear.y = 0.0;
      output.linear.z = 0.0;
      output.angular.x = 0.0;
      output.angular.y = 0.0;
      output.angular.z = 0.0;
      //std::cout << "iteration: " << i << std::endl;
      //std::cout << result << std::endl;
      //std::cout << result.compare("end") << std::endl;
      //if (result.compare("end")==0){
	}
    else{
      //std::cout << result << std::endl;
      output.linear.x = 0.0;
      output.linear.y = 0.0;
      output.linear.z = 0.0;
      output.angular.x = 0.0;
      output.angular.y = 0.0;
      output.angular.z = 0.0;
      pub.publish(output);
      return true;
    }
    ROS_INFO("Finished the burst");
    pub.publish(output);
  }
  count++;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "syncboard_node");
  ros::NodeHandle nh;
  
  // Advertise services
  ros::ServiceServer trigger_service = nh.advertiseService("trigger", trigger_callback); 
  ros::ServiceServer burst_service = nh.advertiseService("burst", burst_callback); 

  // Advertise topics
  ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Twist>("/camera_stitch/cmd_vel", 1);
  ros::Publisher timestamps_pub = nh.advertise<std_msgs::String>("syncbox_timestamps", 1);

  //Synboard stuff
  std::string port;
  int baud;
  nh.param<std::string>("port", port, "/dev/ttyACM1");
  nh.param("baud", baud, 115200);
  syncboard.setTimeout(serial::Timeout::max(), 500, 0, 500, 0);
  syncboard.setPort(port);
  syncboard.setBaudrate(baud);
  syncboard.open();
  handshake();
  while(ros::ok()){
    ros::spinOnce();
    ros::Rate(10).sleep();
  }
  syncboard.write("s");
  return 0;
}
