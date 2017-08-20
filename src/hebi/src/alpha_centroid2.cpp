#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hebi/AlphaCentroid.h"
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
#include <vector>
#include <log4cxx/logger.h>

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

const int num_centroids = 30;
int counter = 0;
float x_loc;
float y_loc;
float z_loc;
int max_inliers;

int othercounter;

float joint21_pos;
float joint22_pos;
float joint23_pos;

struct alpha {
  float x;
  float y;
  float z;
  int inliers;
};

alpha c_tracker [num_centroids];

//std::vector<alpha> c_tracker;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		pub_ = n_.advertise<geometry_msgs::PointStamped>("/kdc/board_point_alpha", 1);
		sub_ = n_.subscribe("/kdc/board_point", 1, &SubscribeAndPublish::callback, this);
        sub_joint_ = n_.subscribe("/joint_states", 1, &SubscribeAndPublish::jointStateCallback, this);
	}

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg)
	{
        ROS_INFO_STREAM("Counter: " << counter);

        if(counter<=(num_centroids-1))
        {
            c_tracker[counter].x = msg->point.x;
            c_tracker[counter].y = msg->point.y;
            c_tracker[counter].z = msg->point.z;

            //c_tracker.push_back(msg->point.x).x;
            //c_tracker.push_back(msg->point.y).y;
            //c_tracker.push_back(msg->point.z).z;

            counter++;
        }

        if (counter==(num_centroids-1))
        {
            ROS_INFO_STREAM("COUNTER IS FILLED!!");
        }

    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        joint21_pos = msg->position[0];
        joint22_pos = msg->position[1];
        joint23_pos = msg->position[2];
    }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Subscriber sub_joint_;
};



bool alphaService(hebi::AlphaCentroid::Request  &req,
          hebi::AlphaCentroid::Response &res)
{

    //alpha c_tracker[30];

    ros::NodeHandle nh;

    //res.alpha_centroid_resp = req.alpha_centroid_req;  //this is a placeholder for now

    ROS_DEBUG_STREAM("Alpha centroid request: " << req.alpha_centroid_req);

    SubscribeAndPublish SAPObject;

    ROS_DEBUG_STREAM("Instantiated the class");

    ros::Publisher publish;
    publish = nh.advertise<sensor_msgs::JointState>("joint_commands", 1);
    sensor_msgs::JointState jointActuate;
    jointActuate.header.stamp = ros::Time::now();
    jointActuate.header.frame_id = "";

    ROS_DEBUG_STREAM("ABOUT TO ENTER THE WHILE-LOOP");

    ros::Rate loop_rate(100);
    othercounter = 0;

    while (othercounter<360)
    {

        ros::spinOnce();
        jointActuate.velocity = {0.0, 0.0, 0.0}; //0.3*sin(othercounter*M_PI/180)};
        publish.publish(jointActuate);
        loop_rate.sleep();
        othercounter++;
    }

    //othercounter = 0;

    while (othercounter<720)
    {

        ros::spinOnce();
        jointActuate.velocity = {0.0, 0.0, 0.0}; //-.3*sin(othercounter*M_PI/180)};
        publish.publish(jointActuate);
        loop_rate.sleep();
        othercounter++;
    }

    jointActuate.position = {joint21_pos, joint22_pos, joint23_pos};
    publish.publish(jointActuate);
    ros::spinOnce();

    ROS_DEBUG_STREAM("CALLBACKS COMPLETED");

    float radius = 0.005;
    int n;
    int i;
    float dist;
    for (n=0; n<(counter); n++)  //loop through every point
    {
        int inliers_counter = 0;
        ROS_DEBUG_STREAM(c_tracker[n].x << "," << c_tracker[n].y << "," << c_tracker[n].z);
        for (i=0; i<(counter); i++)
        {
            dist = sqrt(pow((c_tracker[n].x-c_tracker[i].x),2)+pow((c_tracker[n].z-c_tracker[i].z),2));
            //std::cout << dist << std::endl;
            if (dist < radius)
            {
                inliers_counter++;
            }
        }
        c_tracker[n].inliers = inliers_counter;
        ROS_DEBUG_STREAM("Number of inliers:" << inliers_counter);
        if (n == 0)
        {
            max_inliers = c_tracker[n].inliers;
            x_loc = c_tracker[n].x;
            y_loc = c_tracker[n].y;
            z_loc = c_tracker[n].z;
        }

        else if ((n != 0) && (c_tracker[n].inliers > max_inliers))
        {
            //std::cout << "c_tracker[n].inliers: " << c_tracker[n].inliers << std::endl;
            //std::cout << "c_tracker[n-1].inliers: " << c_tracker[n-1].inliers << std::endl;
            max_inliers = c_tracker[n].inliers;
            x_loc = c_tracker[n].x;
            y_loc = c_tracker[n].y;
            z_loc = c_tracker[n].z;
        }
    }
    ROS_INFO_STREAM("max_inliers: " << max_inliers);
    ROS_INFO_STREAM("loc: " << x_loc << " " << y_loc << " " << z_loc);

    res.alpha_centroid_resp_x = x_loc;
    res.alpha_centroid_resp_y = y_loc;
    res.alpha_centroid_resp_z = z_loc;
    res.alpha_centroid_num_inliers = max_inliers;

    ROS_DEBUG_STREAM("sending back response: " << res.alpha_centroid_resp_x);


    //ugh this is terrible, but it was easier than trying to move everything over to a vector. I just manually clear everything in the struct...
    for(int a=0; a<num_centroids; a++)
    {
        c_tracker[a].x = {};
        c_tracker[a].y = {};
        c_tracker[a].z = {};
        c_tracker[a].inliers = {};
    }

    ROS_DEBUG("What's left in the struct: ");
    for(int a=0; a<num_centroids; a++)
    {
        ROS_DEBUG_STREAM(c_tracker[a].x << ", " << c_tracker[a].y << ", " << c_tracker[a].z << ", " << c_tracker[a].inliers);
    }

    counter = 0;
    max_inliers = 0;
    x_loc = 0;
    y_loc = 0;
    z_loc = 0;

    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "alpha_centroid");

  ros::NodeHandle n;

  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

  ros::ServiceServer service = n.advertiseService("alpha_centroid", alphaService);

  ros::spin();

  return 0;
}
