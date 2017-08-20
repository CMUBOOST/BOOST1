#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "hebi/AlphaCentroid.h"
//#include <geometry_msgs/PointStamped.h>
//#include <sensor_msgs/JointState.h>
//#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/message_filter.h>
//#include <string>
//#include <math.h>
//#include <vector>
#include <log4cxx/logger.h>

//#include "lookup.hpp"
//#include "group.hpp"
//#include "group_command.hpp"

//#include "servoVars.hpp"

//using namespace message_filters;
//using namespace geometry_msgs;
//using namespace sensor_msgs;


//const int num_centroids = 50;
int centroidCounter = 0;
float x_loc;
float y_loc;
float z_loc;
int max_inliers;
/*
int othercounter;

float joint21_pos;
float joint22_pos;
float joint23_pos;
*/
struct alpha {
  float x;
  float y;
  float z;
  int inliers;
};

//alpha c_tracker [num_centroids];

//std::vector<alpha> c_tracker;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		//pub_ = n_.advertise<geometry_msgs::PointStamped>("/kdc/board_point_alpha", 1);
		//sub_ = n_.subscribe("/kdc/board_point", 1, &SubscribeAndPublish::callback, this);
        //sub_joint_ = n_.subscribe("/joint_states", 1, &SubscribeAndPublish::jointStateCallback, this);
	}

	/*
    void callback(const geometry_msgs::PointStamped::ConstPtr& msg)
	{
        ROS_INFO_STREAM("Centroid Counter: " << centroidCounter);

        if(centroidCounter < num_centroids)
        {
            c_tracker[centroidCounter].x = msg->point.x;
            c_tracker[centroidCounter].y = msg->point.y;
            c_tracker[centroidCounter].z = msg->point.z;

            centroidCounter++;
        }

        if (centroidCounter==(num_centroids))
        {
            ROS_DEBUG_STREAM("CENTROID COUNTER IS FILLED!!");
        }

    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        joint21_pos = msg->position[4];
        joint22_pos = msg->position[5];
        joint23_pos = msg->position[6];
    }
    */

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Subscriber sub_joint_;
};



bool alphaService(hebi::AlphaCentroid::Request  &req,
          hebi::AlphaCentroid::Response &res)
{

	/*
    ros::NodeHandle nh;

    ROS_DEBUG_STREAM("Alpha centroid request: " << req.alpha_centroid_req);

    SubscribeAndPublish SAPObject;

    ROS_DEBUG_STREAM("Instantiated the class");
	*/

    /*
    ros::Publisher publish;
    publish = nh.advertise<sensor_msgs::JointState>("joint_commands", 1);
    sensor_msgs::JointState jointActuate;
    jointActuate.header.stamp = ros::Time::now();
    jointActuate.header.frame_id = "";

    ROS_DEBUG_STREAM("ABOUT TO ENTER THE WHILE-LOOP");

    ros::Rate loop_rate(100);
    othercounter = 0;
	*/

    /*
    while ((othercounter<1000) && (centroidCounter!=num_centroids))
    {

        ros::spinOnce();
        jointActuate.velocity = {NAN, NAN, NAN, NAN, 0.0, 0.0, 0.0};
        publish.publish(jointActuate);
        loop_rate.sleep();
        othercounter++;
    }

    ROS_DEBUG_STREAM("CALLBACK COMPLETED");

    //---------------------------------Discards Points Too Far From Line---------------------------------
    std::vector<alpha> truePoints;
    int n;
    int trueCounter = 0;
    for (n=0; n<(centroidCounter); n++)
    {
        std::cout << c_tracker[n].x << "," << c_tracker[n].y << std::endl;
    
        if ((pow(c_tracker[n].x,2) + pow(c_tracker[n].y,2)) < pow(armLengthLimit,2))
        {
            truePoints.push_back(alpha());
            truePoints[trueCounter].x = c_tracker[n].x;
            truePoints[trueCounter].y = c_tracker[n].y;
            trueCounter++;
        }
        else
        {
            float reachlimit = sqrt(pow(c_tracker[n].x,2)+pow(c_tracker[n].y,2));
            ROS_INFO_STREAM("Out of range! " << c_tracker[n].x << ", " << c_tracker[n].y << "... " << reachlimit);
        }
    }

    std::cout << "" << std::endl;


    //---------------------------Calculates Inlier Distances and Optimal Point---------------------------
    float radius = 0.005;
    int i;
    float dist;
    for (n=0; n<(trueCounter); n++)  //loop through every point
    {
        int inliers_counter = 0;
        //std::cout << truePoints[n].x << ", " << truePoints[n].y << std::endl;
        for (i=0; i<(trueCounter); i++)
        {
            dist = sqrt(pow((truePoints[n].x-truePoints[i].x),2)+pow((truePoints[n].y-truePoints[i].y),2));
            //std::cout << dist << std::endl;
            if (dist < radius)
            {
                inliers_counter++;
            }
        }
        truePoints[n].inliers = inliers_counter;
        ROS_DEBUG_STREAM("Number of inliers:" << inliers_counter);
        if (n == 0)
        {
            max_inliers = truePoints[n].inliers;
            x_loc = truePoints[n].x;
            y_loc = truePoints[n].y;
            z_loc = truePoints[n].z;
        }

        else if ((n != 0) && (truePoints[n].inliers > max_inliers))
        {
            max_inliers = truePoints[n].inliers;
            x_loc = truePoints[n].x;
            y_loc = truePoints[n].y;
            z_loc = truePoints[n].z;
        }
    }
    ROS_INFO_STREAM("max_inliers: " << max_inliers);
    ROS_INFO_STREAM("loc: " << x_loc << " " << y_loc << " " << z_loc);
    */

    res.alpha_centroid_resp_x = -0.22; //x_loc;
    res.alpha_centroid_resp_y = -0.35; //y_loc;
    res.alpha_centroid_resp_z = 0; //z_loc;
    res.alpha_centroid_num_inliers = 6; //max_inliers;

    ROS_INFO_STREAM("sending back response: " << res.alpha_centroid_resp_x);
    //------------------------------End of Calculating Inliers and Optimal Point---------------------------

    /*
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
	*/

    /*
    centroidCounter = 0;
    max_inliers = 0;
    x_loc = 0;
    y_loc = 0;
    z_loc = 0;
	*/

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
