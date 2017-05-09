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
#include <unistd.h>


using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

int globalCounter = 0;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_old(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_full(new pcl::PointCloud<pcl::PointXYZRGB>);
typedef boost::circular_buffer<sensor_msgs::PointCloud2> circular_buffer;
circular_buffer cb(5);


void pairAlign (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{

  	ROS_INFO_STREAM("cloud_src size:" << cloud_src->size());
  	ROS_INFO_STREAM("cloud_tgt size:" << cloud_tgt->size());  

  	// Downsample for consistency and speed
  	// \note enable this for large datasets
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);
  	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  	if (downsample)
  	{
    	grid.setLeafSize (0.01, 0.01, 0.01);
   		grid.setInputCloud (cloud_src);
    	grid.filter (*src);

    	grid.setInputCloud (cloud_tgt);
    	grid.filter (*tgt);
  	}
  	else
  	{
    	src = cloud_src;
    	tgt = cloud_tgt;
  	}

  	ROS_INFO_STREAM("src size after downsample:" << src->size());
  	ROS_INFO_STREAM("tgt size after downsample:" << tgt->size());  

  	// Compute surface normals and curvature
  	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointNormal>);
  	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::PointNormal>);

  	ROS_DEBUG("About to estimate normals");
  	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norm_est;
  	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  	norm_est.setSearchMethod (tree);
  	norm_est.setKSearch (30);
  
  	norm_est.setInputCloud (src);
  	norm_est.compute (*points_with_normals_src);
  	pcl::copyPointCloud (*src, *points_with_normals_src);

  	norm_est.setInputCloud (tgt);
  	norm_est.compute (*points_with_normals_tgt);
  	pcl::copyPointCloud (*tgt, *points_with_normals_tgt);
  	ROS_DEBUG("Normals estimated");

  	// Align
  	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> reg;
  	reg.setTransformationEpsilon (1e-6); //1e-6
  	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
  	// Note: adjust this based on the size of your datasets
  	reg.setMaxCorrespondenceDistance (0.1);  //originally 0.15

  	reg.setInputSource (points_with_normals_src);
  	reg.setInputTarget (points_with_normals_tgt);

  	// Run the same optimization in a loop and visualize the results
  	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  
  	//PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  	pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;
  	reg.setMaximumIterations (2);

  	int i=0;
  	while (reg.getMaxCorrespondenceDistance() > 0.01)
  	{
    	i = i+1;

    	// save cloud for visualization purpose
    	points_with_normals_src = reg_result;

    	// Estimate
    	reg.setInputSource (points_with_normals_src);
    	reg.align (*reg_result);

    	//accumulate transformation between each Iteration
    	Ti = reg.getFinalTransformation () * Ti;

    	//if the difference between this transformation and the previous one
    	//is smaller than the threshold, refine the process by reducing
    	//the maximal correspondence distance
    	if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      		reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.01);

    	prev = reg.getLastIncrementalTransformation ();

    	// visualize current state
    	//showCloudsRight(points_with_normals_tgt, points_with_normals_src);   
  	}

  
  	std::cout << "Iterations: " << i << std::endl;

  	// Get the transformation from target to source
  	targetToSource = Ti.inverse();

  	// Transform target back in source frame
  	pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  	//add the source to the transformed target
  	*output += *cloud_src;
  
  	final_transform = targetToSource;
 }






void filterStitch(boost::circular_buffer<sensor_msgs::PointCloud2> ringbuffer)
{

	for (int i=0; i<5; i++){
		ROS_INFO_STREAM("header: \n" << ringbuffer[i].header);


//----------Convert PointCloud2 to PCL------------------
	    pcl::PCLPointCloud2 pcl_pc2;
	    pcl_conversions::toPCL(ringbuffer[i],pcl_pc2);
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

//-----------Initialize Empty Point Clouds-------------
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);

	    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

//---------Create a conditional removal filter to remove black pixels---------
	    int rMax = 255;
	    int rMin = 15;
	    int gMax = 255;
	    int gMin = 15;
	    int bMax = 255;
	    int bMin = 15;
	    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
	    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
	    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin)));
	    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax)));
	    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
	    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
	    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin)));

	    // build the filter
	    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
	    condrem.setInputCloud (temp_cloud);
	    condrem.setKeepOrganized(true);
	      
	    // apply filter
	    condrem.filter (*cloud_filtered);

		ROS_INFO_STREAM("Cloud_Filtered size:" << cloud_filtered->size());

		//Passthrough filter	
	    pcl::PassThrough<pcl::PointXYZRGB> pass;
	    pass.setInputCloud (cloud_filtered);
	    pass.setFilterFieldName ("z");
	    pass.setFilterLimits (0, 0.8);
	    pass.filter (*cloud_filtered);

		std::stringstream ss2;
		ss2 << "testCloud" << globalCounter <<".ply";
		pcl::PLYWriter writer;
	    writer.write<pcl::PointXYZRGB> (ss2.str(), *cloud_filtered, false);

		//-----------------insert statistical outlier removal here if needed----------------
		//
		//

	    std::vector<int> indices;
	    pcl::removeNaNFromPointCloud(*cloud_filtered,*cloud_filtered, indices);

		ROS_INFO_STREAM("Cloud_Filtered after NAN removal:" << cloud_filtered->size());
	
	    if (i>0)
	    {
	        source = cloud_filtered_old;
			target = cloud_filtered;

	        // Add visualization data
	        //showCloudsLeft(source, target);

	        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
	        ROS_INFO ("Aligning..."); 


			//Stitch::pairAlign (temp, pairTransform, true);        
			pairAlign (source, target, temp, pairTransform, true);
	        
	        //transform current pair into the global transform
	        pcl::transformPointCloud (*temp, *result, GlobalTransform);

	        //update the global transform
	        GlobalTransform = GlobalTransform * pairTransform;

			*cloud_full+=*result;
	    }
	  
	    //ROS_INFO_STREAM("Cloud vec size: " << cloud_vec.size());

	    copyPointCloud(*cloud_filtered, *cloud_filtered_old); 

		ROS_INFO_STREAM("cloud_filtered_old size: " << cloud_filtered_old->size());

	    temp_cloud->clear();

		ROS_INFO_STREAM("Global Counter: " << globalCounter);

		if (i == 4){
			ROS_INFO("Saving the stitched cloud!");
			std::stringstream ss3;
			writer.write<pcl::PointXYZRGB> ("StitchedCloud.ply", *cloud_full, false);
		}
	}
}

void callback(const boost::shared_ptr<const nav_msgs::Odometry>& odomPtr, const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudPtr)
{

	//std::cout << globalCounter << std::endl;
	//ROS_INFO_STREAM("header: \n" << cloudPtr->header);
	
	cb.push_back(*cloudPtr);

	std::cout << "Size of cb: " << cb.size() << std::endl;

	globalCounter++;

	if (cb.size() == 5){
		filterStitch(cb);
	}
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
