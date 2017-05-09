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

float joint21_pos;
float joint22_pos;
float joint23_pos;

struct alpha {
  float x;
  float y;
  float z;
  int inliers;
} apple [num_centroids];

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

        if(counter<=(num_centroids-1))
        {
            apple[counter].x = msg->point.x;
            apple[counter].y = msg->point.y;
            apple[counter].z = msg->point.z;
            counter++;
            std::cout << counter << std::endl;
        }

        if (counter==(num_centroids-1))
        {
            std::cout << "COUNTER IS FILLED!!"  <<std::endl;
            float radius = 0.005;
            int n;
            int i;
            float dist;
            for (n=0; n<(num_centroids-1); n++)  //loop through every point
            {
                int inliers_counter = 0;
                std::cout << apple[n].x << "," << apple[n].y << "," << apple[n].z << std::endl;
                for (i=0; i<(num_centroids-1); i++)
                {
                    dist = sqrt(pow((apple[n].x-apple[i].x),2)+pow((apple[n].z-apple[i].z),2));
                    //std::cout << dist << std::endl;
                    if (dist < radius)
                    {
                        inliers_counter++;
                    }
                }
                apple[n].inliers = inliers_counter;
                //std::cout << "Number of inliers:" << inliers_counter << std::endl;
                if (n == 0)
                {
                    max_inliers = apple[n].inliers;
                    x_loc = apple[n].x;
                    y_loc = apple[n].y;
                    z_loc = apple[n].z;
                }

                else if ((n != 0) && (apple[n].inliers > max_inliers))
                {
                    //std::cout << "apple[n].inliers: " << apple[n].inliers << std::endl;
                    //std::cout << "apple[n-1].inliers: " << apple[n-1].inliers << std::endl;
                    max_inliers = apple[n].inliers;
                    x_loc = apple[n].x;
                    y_loc = apple[n].y;
                    z_loc = apple[n].z;
                }
            }
            std::cout << "max_inliers: " << max_inliers <<std::endl;
            std::cout << "loc: " << x_loc << " " << y_loc << " " << z_loc << " " << std::endl;
        }

        else if (counter <= num_centroids)
        {
            geometry_msgs::PointStamped alphaPoint;
            alphaPoint.header.stamp = ros::Time::now();
            alphaPoint.header.frame_id = "";

            alphaPoint.point.x = x_loc;
            alphaPoint.point.y = y_loc;
            alphaPoint.point.z = z_loc;
            pub_.publish(alphaPoint);
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

    ros::NodeHandle nh;

    //res.alpha_centroid_resp = req.alpha_centroid_req;  //this is a placeholder for now

    ROS_INFO_STREAM("Alpha centroid request: " << req.alpha_centroid_req);

    SubscribeAndPublish SAPObject;

    ROS_INFO_STREAM("Instantiated the class");

    ros::Publisher publish;
    publish = nh.advertise<sensor_msgs::JointState>("joint_commands", 1);
    sensor_msgs::JointState jointActuate;
    jointActuate.header.stamp = ros::Time::now();
    jointActuate.header.frame_id = "";

    ROS_INFO_STREAM("ABOUT TO ENTER THE WHILE-LOOP");

    ros::Rate loop_rate(100);
    int othercounter = 0;

    while (othercounter<360)
    {

        ros::spinOnce();
        jointActuate.velocity = {0.0, 0.0, 0.3*sin(othercounter*M_PI/180)};
        publish.publish(jointActuate);
        loop_rate.sleep();
        othercounter++;
    }

    othercounter = 0;

    while (othercounter<360)
    {

        ros::spinOnce();
        jointActuate.velocity = {0.0, 0.0, -.3*sin(othercounter*M_PI/180)};
        publish.publish(jointActuate);
        loop_rate.sleep();
        othercounter++;
    }

    jointActuate.position = {joint21_pos, joint22_pos, joint23_pos};
    publish.publish(jointActuate);
    ros::spinOnce();

    res.alpha_centroid_resp_x = 0.1; //THIS IS JUST BECAUSE I CHANGED THE SRV FILE!!!

    ROS_INFO_STREAM("sending back response: " << res.alpha_centroid_resp_x);

    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "alpha_centroid");

  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("alpha_centroid", alphaService);

  ros::spin();

  return 0;
}
