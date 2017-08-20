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
#include <unistd.h>

using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

int globalCounter = 0;
//int whileCounter = 0;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_old(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_full(new pcl::PointCloud<pcl::PointXYZRGB>);
//std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vec;

class Stitch
{
public:
	Stitch();
	void calcError(const boost::shared_ptr<const nav_msgs::Odometry>& odomPtr, const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudPtr);
	
private:

	ros::NodeHandle mNode;
	ros::Publisher mErrorPub;
};

Stitch::Stitch()
{
	int buffer = 10;
	mErrorPub = mNode.advertise<std_msgs::String>("chatter", buffer);
}


void Stitch::calcError(const boost::shared_ptr<const nav_msgs::Odometry>& odomPtr, const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudPtr)
{

	ROS_INFO_STREAM("header: \n" << cloudPtr->header);

	std::cout << "Global Counter: " << globalCounter << std::endl;
	
	usleep(1000000);

	globalCounter++;


}

bool cloudStitch(image_stitch::CloudStitch::Request  &req,
          image_stitch::CloudStitch::Response &res)
{
    ros::NodeHandle nh;
    nh.setCallbackQueue(&my_callback_queue);

    //res.cloud_stitch_resp = req.cloud_stitch_req;

    //ROS_INFO_STREAM("Cloud stitch request: " << req.cloud_stitch_req);

	Stitch* pStitch = new Stitch();

  	message_filters::Subscriber<nav_msgs::Odometry> odomSub(nh, "/wheel_encoder/odom", 10);
  	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub(nh, "/multisense/image_points2_color", 10);

  	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
  	Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), odomSub, cloudSub);
  	sync.registerCallback(boost::bind(&Stitch::calcError, pStitch, _1, _2));

	double begin = ros::Time::now().toSec();
	double end;
	double timer = 0;
	static const double timeout = 0.01;

	while(globalCounter<5 && ros::ok()){ 
		ros::CallbackQueue::callAvailable(ros::WallDuration(timeout)); 	
		ros::spinOnce();
		//my_callback_queue.callOne(ros::WallDuration());
		//ros::AsyncSpinner spinner(0, &my_callback_queue);
  		//spinner.start();
	}

    //ROS_INFO_STREAM("sending back response: " << res.cloud_stitch_resp);

	globalCounter = 0;

    return true;
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "stitcher_server");

  	ros::NodeHandle n;

  	ros::ServiceServer service = n.advertiseService("cloud_stitch", cloudStitch);
  	ROS_INFO("Ready to pull in point clouds!");

	ros::spin();

	return 0;
}
