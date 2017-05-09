#include <ros/ros.h>
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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include "image_stitch/CloudStitch.h"
#include <string.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include <unistd.h>


using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

int globalCounter = 0;
typedef boost::circular_buffer<sensor_msgs::PointCloud2> circular_buffer;
circular_buffer cb(5);

void stitcher(boost::circular_buffer<sensor_msgs::PointCloud2> ringbuffer)
{

	for (int i=0; i<5; i++){
		ROS_INFO_STREAM("header: \n" << ringbuffer[i].header);
	}
}

void callback(const boost::shared_ptr<const nav_msgs::Odometry>& odomPtr, const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudPtr)
{

	std::cout << globalCounter << std::endl;
	ROS_INFO_STREAM("header: \n" << cloudPtr->header);
	
	cb.push_back(*cloudPtr);

	// Check if cloudPtr is full
	std::cout << "Size of cb: " << cb.size() << std::endl;

	if (cb.size() == 5){
		stitcher(cb);
	}

	globalCounter++;
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "stitcher_server");

  	ros::NodeHandle nh;

  	message_filters::Subscriber<nav_msgs::Odometry> odomSub(nh, "/wheel_encoder/odom", 10);
  	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub(nh, "/multisense/image_points2_color", 10);

  	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;

  	Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), odomSub, cloudSub);
  	sync.registerCallback(boost::bind(&callback, _1, _2));

	while(globalCounter<5 && ros::ok()){ 
		ros::spinOnce();
	}

	return 0;
}
