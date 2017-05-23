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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include "image_stitch/CloudStitch.h"
#include <string.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_srvs/Trigger.h"
#include <unistd.h>

using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

class Stitch
{

public:
	Stitch();
	void callback (const boost::shared_ptr<const nav_msgs::Odometry>& odomPtr, const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudPtr);
	
private:
	ros::NodeHandle mNode;
	ros::Publisher mErrorPub;
	Eigen::Matrix4f GlobalTransform, pairTransform;
};

Stitch::Stitch()
{
	int buffer = 10;
	this->mErrorPub = mNode.advertise<std_msgs::String>("chatter", buffer);
	GlobalTransform = Eigen::Matrix4f::Identity ();
}

void Stitch::callback(const boost::shared_ptr<const nav_msgs::Odometry>& odomPtr, const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudPtr)
{
	//std::cout << "Wait I'm in here!!" << std::endl;
	ROS_INFO_STREAM("I'm IN HERE!!"); 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stitcher_server");

  	ros::NodeHandle nh;

	Stitch* pStitch = new Stitch;

  	message_filters::Subscriber<nav_msgs::Odometry> odomSub(nh, "/wheel_encoder/odom", 10);
  	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub(nh, "/multisense/image_points2_color", 10);

  	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;

  	Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), odomSub, cloudSub);
  	sync.registerCallback(boost::bind(&Stitch::callback, pStitch, _1, _2));

	ros::spin();
	
	return 0;
}
