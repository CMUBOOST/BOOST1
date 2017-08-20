#include <ros/ros.h>
#include <log4cxx/logger.h>
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
#include <sstream> 
#include <iostream>
#include <stdio.h>      /* printf */
#include <stdlib.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

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

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <exception>

#include "image_stitch/AlphaCentroid.h"
#include "arm_executive/ArmInitStitch.h"
#include "gripper/GripperClose.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

static const std::string OPENCV_WINDOW = "Image window";

class Stitch
{

// BEG .h
public:
	Stitch();
	void callback (const boost::shared_ptr<const sensor_msgs::Image>& imagePtr, const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudPtr);
	//void on_low_r_thresh_trackbar(int, void *);
	//void on_high_r_thresh_trackbar(int, void *);

// ---	
private:
	ros::NodeHandle mNode;
	ros::Publisher cloudPub;
	ros::Publisher imagePub;	
	ros::Publisher markerPub;
};
// END .h

Stitch::Stitch()
{
	int buffer = 10;
	//this->cloudPub = mNode.advertise<std_msgs::String>("stitched_cloud", buffer);
	this->cloudPub = mNode.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("stitched_cloud", 1);
	this->imagePub = mNode.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("stitched_cloud", 1);	
	this->markerPub = mNode.advertise<visualization_msgs::Marker>("visualization_marker", 1);

}

void Stitch::callback(const boost::shared_ptr<const sensor_msgs::Image>& imagePtr, const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudPtr)
{

	std::cout << "Yeah!" << std::endl;

	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(imagePtr, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	/*
	int dstWidth = cv_ptr->image.cols;
    int dstHeight = cv_ptr->image.rows * 2;
    cv::Mat dst = cv::Mat(dstHeight, dstWidth, CV_8UC3, cv::Scalar(0,0,0));
    cv::Rect roi(cv::Rect(0,0,cv_ptr->image.cols, cv_ptr->image.rows));
    cv::Mat targetROI = dst(roi);
    cv_ptr->image.copyTo(targetROI);
    targetROI = dst(cv::Rect(0,cv_ptr->image.rows,cv_ptr->image.cols, cv_ptr->image.rows));
	*/

	cv::Mat output, hsv_img;
	/*
	cv::cvtColor(cv_ptr->image, hsv_img, COLOR_BGR2HSV);
	cv::inRange(hsv_img, cv::Scalar(30, 25, 60), cv::Scalar(82, 130, 255), output);
	*/
	cv::inRange(cv_ptr->image, cv::Scalar(36, 45, 26), cv::Scalar(255, 255, 255), output);

	cv::imshow(OPENCV_WINDOW, output);

    cv::waitKey(3);

}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "stitcher_server");

  	ros::NodeHandle nh;

	Stitch* pStitch = new Stitch;

  	message_filters::Subscriber<sensor_msgs::Image> imageSub(nh, "/multisense/left/image_rect_color", 20);
  	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub(nh, "/multisense/image_points2_color", 20);

  	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

  	Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), imageSub, cloudSub);
  	sync.registerCallback(boost::bind(&Stitch::callback, pStitch, _1, _2));

	ros::spin();
	
	return 0;
}