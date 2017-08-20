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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>

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
#include "Vec3.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <exception>

#include "image_stitch/AlphaCentroid.h"
#include "arm_executive/ArmInitStitch.h"
#include "gripper/GripperClose.h"

using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

typedef boost::circular_buffer<sensor_msgs::PointCloud2> circular_buffer;
typedef boost::circular_buffer<nav_msgs::Odometry> circular_buffer_odom;

struct MyPointType
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  int layer;
  float weight;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, layer, layer)
                                   (float, weight, weight)
)

class Stitch
{

// BEG .h
public:
	Stitch();
	void filterStitch(boost::circular_buffer<sensor_msgs::PointCloud2> ringbuffer, boost::circular_buffer<nav_msgs::Odometry> odom_buffer);
	void pairAlign (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);
	void callback (const boost::shared_ptr<const nav_msgs::Odometry>& odomPtr, const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudPtr, const boost::shared_ptr<const sensor_msgs::JointState>& jointPtr);
	//bool cloudStitch(image_stitch::CloudStitch::Request  &req, image_stitch::CloudStitch::Response &res);
	bool cloudStitch(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
	void slice(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_full, Eigen::Matrix4f GlobalTransform_odom);
	std::vector<std::vector<float> > hotspotfilterFunction(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int n);
	std::vector<std::vector<float> > sphereLogic2(std::vector<std::vector<std::vector<float> > > sphere_locs_3d);
	//bool alphaService(image_stitch::AlphaCentroid::Request  &req, image_stitch::AlphaCentroid::Response &res);
// ---	
private:
	ros::NodeHandle mNode;
	ros::Publisher cloudPub;
	ros::Publisher markerPub;
	circular_buffer cb;
	circular_buffer_odom cb_odom;
	Eigen::Matrix4f GlobalTransform, pairTransform;
	Eigen::Matrix4f GlobalTransform_odom;	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_full;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_old;

	double arm_height;
	float centroid_x;
	float centroid_y;
	const static float cam_x_offset = 0.255; //.265
	const static float cam_z_offset = 0.015;	
	const static float arm_extended = 0.565;

	//---------Hardcoded Centroids at each layer--------
	const static int num_centroids_layer = 35;

	//---------Number of Images------------
	const static int num_images = 5;

	//--------Stitching Variables-----------
	const static float pairAlign_leafsize = 0.01;
	const static int pairAlign_normest_neighbors = 30;
	const static float pairAlign_tform_eps = 1e-6;
	const static int pairAlign_max_iterations = 2;
	const static float pairAlign_correspondenceDistance = 0.1;
	const static float pairAlign_correspondenceDistance_cutoff = 0.01;

	//--------RGB Filtering Variables-------- 	
	const static int rMax = 255;
	const static int rMin = 15; //31;
	const static int gMax = 255;
	const static int gMin = 15; //22;
	const static int bMax = 255;
	const static int bMin = 15; //22;

	//----------Heat Map Filtering----------
	const static int noise_threshold = 400;//40;
	const static int hotspot_threshold = 800;

	//---------Sphere/Cylinder Logic--------
	const static float z_threshold = 0.02; //0.02
	const static float x_threshold = 0.02; //0.02
	const static float RANSAC_threshold = 0.025;

	//------------Cloud Slicing------------
	const static int num_slices = 8;
	const static float slice_leafsize = 0.001;
	//const static int slice_normal_radius = 0.01;
	const static float bottom_start = 0.3;
	const static float middle_start = 0.4;
	const static float top_start = 0.5;	
	const static int num_spheres = 100; //can't remember if this is the total number of spheres or for each layer...
	const static float far_reach_reduction = 0.4; //this is a ridiculous variable to prevent the arm reaching over to the camera region

	//------------Cylinder Detection----------
	const static float cylinder_dist_threshold = 0.001;	

	std::vector<std::vector<float> > centroids;
};
// END .h

Stitch::Stitch()
{
	int buffer = 10;
	//this->cloudPub = mNode.advertise<std_msgs::String>("stitched_cloud", buffer);
	this->cloudPub = mNode.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("stitched_cloud", 1);
	this->markerPub = mNode.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	this->cb = circular_buffer(num_images);
	this->cb_odom = circular_buffer_odom(num_images);
	GlobalTransform = Eigen::Matrix4f::Identity ();
	GlobalTransform_odom = Eigen::Matrix4f::Identity ();	

	cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_filtered_old.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_full.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void Stitch::pairAlign (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{

  	// Downsample for consistency and speed
  	// \note enable this for large datasets
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);
  	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  	if (downsample)
  	{
    	grid.setLeafSize (pairAlign_leafsize, pairAlign_leafsize, pairAlign_leafsize);
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

  	// Compute surface normals and curvature
  	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointNormal>);
  	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::PointNormal>);

  	ROS_DEBUG("About to estimate normals");
  	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norm_est;
  	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  	norm_est.setSearchMethod (tree);
  	norm_est.setKSearch (pairAlign_normest_neighbors);
  
  	norm_est.setInputCloud (src);
  	norm_est.compute (*points_with_normals_src);
  	pcl::copyPointCloud (*src, *points_with_normals_src);

  	norm_est.setInputCloud (tgt);
  	norm_est.compute (*points_with_normals_tgt);
  	pcl::copyPointCloud (*tgt, *points_with_normals_tgt);
  	ROS_DEBUG("Normals estimated");

  	// Align
  	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> reg;
  	reg.setTransformationEpsilon (pairAlign_tform_eps); //1e-6
  	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
  	// Note: adjust this based on the size of your datasets
  	reg.setMaxCorrespondenceDistance (pairAlign_correspondenceDistance);  //originally 0.15

  	reg.setInputSource (points_with_normals_src);
  	reg.setInputTarget (points_with_normals_tgt);

  	// Run the same optimization in a loop and visualize the results
  	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  
  	//PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  	pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;
  	reg.setMaximumIterations (pairAlign_max_iterations);

  	int i=0;
  	while (reg.getMaxCorrespondenceDistance() > pairAlign_correspondenceDistance_cutoff)
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

  
  	ROS_INFO_STREAM("Iterations: " << i);

  	// Get the transformation from target to source
  	targetToSource = Ti.inverse();

  	// Transform target back in source frame
  	pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  	//add the source to the transformed target
  	*output += *cloud_src;
  
  	final_transform = targetToSource;

  	std::cout << final_transform <<std::endl;
  	exit (EXIT_FAILURE);
 }


void Stitch::filterStitch(boost::circular_buffer<sensor_msgs::PointCloud2> ringbuffer, boost::circular_buffer<nav_msgs::Odometry> odom_buffer)
{

	for (int i=(num_images-1); i>-1; i--){  //iterates from 4 to zero
		ROS_DEBUG_STREAM("header: \n" << ringbuffer[i].header);
		ROS_DEBUG_STREAM("ODOMETRY: \n" << odom_buffer[i].pose.pose.position);
		double dist_traveled = odom_buffer[i].pose.pose.position.x-odom_buffer[(num_images-1)].pose.pose.position.x;
		ROS_DEBUG_STREAM("DIST TRAVELED: \n" << dist_traveled);

//----------Convert PointCloud2 to PCL------------------
	    pcl::PCLPointCloud2 pcl_pc2;
	    pcl_conversions::toPCL(ringbuffer[i],pcl_pc2);
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

//-----------Initialize Empty Point Clouds-------------
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);

	    ROS_DEBUG_STREAM("Eigen");

//---------Create a conditional removal filter to remove black pixels---------
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
		ROS_DEBUG_STREAM("Cloud_Filtered size:" << cloud_filtered->size());

		//Passthrough filter	
	    pcl::PassThrough<pcl::PointXYZRGB> pass;
	    pass.setInputCloud (cloud_filtered);
	    pass.setFilterFieldName ("z");
	    pass.setFilterLimits (0, 0.8);
	    pass.filter (*cloud_filtered);

	    //Adjust cloud location based on wheel odometry
	    Eigen::Matrix4f odom_transform = Eigen::Matrix4f::Identity();
	    odom_transform (0,3) = dist_traveled; //TODO: change this to be x,y,z
	    pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, odom_transform);

		std::stringstream ss2;
		ss2 << "testCloud" << i <<".ply";
		pcl::PLYWriter writer;
	    writer.write<pcl::PointXYZRGB> (ss2.str(), *cloud_filtered, false);

		//-----------------insert statistical outlier removal here if needed----------------
		//
		//

	    std::vector<int> indices;
	    pcl::removeNaNFromPointCloud(*cloud_filtered,*cloud_filtered, indices);

		//ROS_INFO_STREAM("Cloud_Filtered after NAN removal:" << cloud_filtered->size());
	
	    if (i<(num_images-1))
	    {
	        source = cloud_filtered_old;
			target = cloud_filtered;

	        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
	        ROS_INFO ("Aligning...");  

			//This takes in (old cloud, new cloud, empty cloud, empty matrix)
			//Converts empty cloud to stitched cloud, and pairTransform to the tf between the two clouds)      
			Stitch::pairAlign (source, target, temp, pairTransform, true);
	        
	        //transform current pair into the global transform
	        //Takes in (stitched cloud, cloud to recieve, and global eigen matrix) 
	        pcl::transformPointCloud (*temp, *result, GlobalTransform);

	        //update the global transform
	        GlobalTransform = GlobalTransform * pairTransform;

	        //update the global transform with odom
	        GlobalTransform_odom = GlobalTransform * odom_transform;

	        ROS_DEBUG_STREAM("Global Tform: " << GlobalTransform_odom);	        

			*cloud_full+=*result;
	    }
	  
	    copyPointCloud(*cloud_filtered, *cloud_filtered_old); 
		ROS_DEBUG_STREAM("cloud_filtered_old size: " << cloud_filtered_old->size());

	    temp_cloud->clear();

		if (i == 0){
			ROS_DEBUG("Saving the stitched cloud! _stitcher10");
			writer.write<pcl::PointXYZRGB> ("StitchedCloud_new.ply", *cloud_full, false);
			ROS_DEBUG_STREAM("Cloud size: " << cloud_full->size());
		}
	}
	ROS_DEBUG("Exiting filterStitch");
}


void Stitch::callback(const boost::shared_ptr<const nav_msgs::Odometry>& odomPtr, const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudPtr, const boost::shared_ptr<const sensor_msgs::JointState>& jointPtr)
{

	arm_height = jointPtr->position[5];

	ROS_DEBUG_STREAM(odomPtr->pose.pose.position);		

	cb.push_back(*cloudPtr);
	cb_odom.push_back(*odomPtr);	

	ROS_INFO_STREAM("Size of cb: " << cb.size());

	if (cb.size() == num_images){
		std::clock_t begin = clock();
		Stitch::filterStitch(cb, cb_odom);
		//this->filterStitch(cb);

		// Publish stitched cloud
		ROS_DEBUG_STREAM("Time: " << ros::Time::now().toNSec());
  		//cloud_full->header.stamp = ros::Time::now().toNSec();
  		cloud_full->header.frame_id = "multisense/left_camera_optical_frame"; //"multisense/head";
  		cloudPub.publish (cloud_full);

		pcl::PLYWriter writer;
		writer.write<pcl::PointXYZRGB> ("StitchedCloud_debug.ply", *cloud_full, false);

  		ROS_DEBUG_STREAM("--------------DONE STITCHING!!---------------");
		std::clock_t stitching_time = clock();  		

		// Estimate stalk locations
		ROS_DEBUG("Entering slice function");		
		Stitch::slice(cloud_full, GlobalTransform_odom);

		ROS_DEBUG_STREAM("-------------------------DONE SLICING!!------------------------");
		std::clock_t end = clock();
  		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		ROS_INFO_STREAM("-------------------------TOTAL ELAPSED TIME------------------------");  		
		ROS_INFO_STREAM("-------------------------" << elapsed_secs << " SECONDS-----------------------");  

		double stitch_secs = double(stitching_time - begin) / CLOCKS_PER_SEC;
		double detection_secs = double(end - stitching_time) / CLOCKS_PER_SEC;
		ROS_INFO_STREAM("STITCHING TIME: " << stitch_secs << ",   DETECTION TIME: " << detection_secs); 		

		// Close the gripper
		ros::ServiceClient gripperclient = mNode.serviceClient<gripper::GripperClose>("gripper_close");	
		gripper::GripperClose srv;
	  	srv.request.gripper_close_req = "close";
	  	if(gripperclient.call(srv))
	  	{
	    	ROS_DEBUG_STREAM("Response: " << srv.response.gripper_close_resp);
	  	}
	  	else
	  	{
	    	ROS_ERROR("Failed to call service");
	  	}

		// Clear all variables
		cb.clear();
		cloud_filtered->clear();
		cloud_filtered_old->clear();
		cloud_full->clear();	
		GlobalTransform = Eigen::Matrix4f::Identity ();
		GlobalTransform_odom = Eigen::Matrix4f::Identity ();								
	}
}

std::vector<std::vector<float> > Stitch::hotspotfilterFunction(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int n)
{

	int gnu_counter = n;
	ROS_DEBUG_STREAM("gnu_counter: " << gnu_counter);
//-------------------Apply filtering based on normals and x-density-----------------------

	pcl::PointNormal minPt, maxPt;
	pcl::getMinMax3D (*cloud, minPt, maxPt);

	ROS_DEBUG("got getMinMax3D");	

	float z_distr = maxPt.z-minPt.z;
	//float y_distr = maxPt.y-minPt.y; 
	float x_distr = maxPt.x-minPt.x; 

	int no_of_rows = int(x_distr*100)+1; 
	int no_of_cols = int(z_distr*100)+1;
	int initial_value = 0;

	std::vector<std::vector<int> > matrix;
	std::vector<std::vector<float> > hotspots;
	std::vector<std::vector<float> > hotspots_gnuplot; //this is for debugging!
	std::vector<std::vector<float> > matrix_normals;
	std::map <std::string, std::vector<int> > point_indices; //this is a map that will be filled with a string of the matrix location and the associated point indices
	ROS_DEBUG("Initialized vectors");
	
	matrix.resize(no_of_rows, std::vector<int>(no_of_cols, initial_value));
	matrix_normals.resize(no_of_rows, std::vector<float>(no_of_cols, initial_value));
	hotspots.resize(num_centroids_layer, std::vector<float>(3, initial_value)); //this was 10!!
	hotspots_gnuplot.resize(num_centroids_layer, std::vector<float>(3, initial_value)); //this is for debugging!

	// Loop through every point in the cloud, adding to the histogram	
	for(int i=0; i<cloud->size(); i++)
	{
		int scatter_x = -int(((*cloud)[i].x*-100) + (minPt.x)*100); //this is the y-location of the point
		int scatter_z = -int(((*cloud)[i].z*-100) + (minPt.z)*100); //this is the z-location of the point

		matrix[scatter_x][scatter_z] = matrix[scatter_x][scatter_z]+1; //add a count to that cell location
		matrix_normals[scatter_x][scatter_z] = matrix_normals[scatter_x][scatter_z]+std::abs((*cloud)[i].normal_y); //MJ UNSURE OF THIS!! //add z-normal to that cell location

		//HERE I KEEP TRACK OF WHICH POINT INDICES ARE ASSOCIATED WITH WHICH LOCATION
		std::stringstream ss;
		ss << scatter_x << scatter_z;
		point_indices[ss.str()].push_back(i);
	}

	ROS_DEBUG("Done with loop");

	std::ofstream outFile;
	std::ofstream otherFile;
	//std::stringstream gnu_name;
	//gnu_name << "heatmap_" << gnu_counter << ".dat";
	//const char *gnu_name = "string Literal";
	char gnu_name[12];   // array to hold the result.
	strcpy(gnu_name,"heatmap_"); // copy string one into the result.
	char integer_string[32];
	sprintf(integer_string, "%d", gnu_counter);
	strcat(gnu_name, integer_string); // append string two to the result.
	strcat(gnu_name, ".dat");
	ROS_DEBUG("Named the .dat file");

	//----just for debugging-----
	char other_name[25];   // array to hold the result.
	strcpy(other_name,"heatmap_other_"); // copy string one into the result.
	char other_integer_string[32];
	sprintf(other_integer_string, "%d", gnu_counter);
	strcat(other_name, other_integer_string); // append string two to the result.
	strcat(other_name, ".dat");
	otherFile.open(other_name);
	//-----end of debugging------

	outFile.open(gnu_name);
	gnu_counter++;

	float nanInsert = std::numeric_limits<float>::quiet_NaN();
	int I;
	int J;
	int counter = 0;
	ROS_DEBUG("Initialized variables");

	// This changes the histogram to be weighted by the normals as well
	/*
	for(int i=0; i<no_of_rows; i++)
	{
		for(int j=0; j<no_of_cols; j++) 
		{
			matrix_normals[i][j] = std::abs(matrix_normals[i][j])/(matrix[i][j]);
			matrix[i][j] = matrix[i][j]*(1-matrix_normals[i][j]);
		}
	}

	ROS_DEBUG("Weighted the heatmap based on normals");	
	*/

	// This is just to fill in the .dat file that's useful for debugging
	for(int i=0; i<no_of_rows; i++)
	{
		for(int j=0; j<no_of_cols; j++) 
		{
			if(matrix[i][j]>0) {
				outFile << i << " " << j << " " << matrix[i][j] << "\n";
			}
			else{
				outFile << i << " " << j << " " << 0 << "\n";			
			}
		}
	}

	// THIS TAKES THE AVERAGE NORMAL VECTOR FOR EACH VOXEL AND THEN DECIDES TO KEEP OR TOSS THE ELEMENTS OF THE VOXEL
	for(int i=0; i<no_of_rows; i++)
	{
		for(int j=0; j<no_of_cols; j++) 
		{
			ROS_DEBUG("In the inner loop");

			if ((matrix[i][j]>hotspot_threshold)) { //&& (matrix_normals[i][j])<0.5) {  //i.e. find a random hotspot

				ROS_INFO_STREAM("Initializing queue: " << i << " " << j << ", Value: " << matrix[i][j] << ", No_of_cols: " << no_of_cols << ", No_of_rows: " << no_of_rows);
				std::vector<std::vector<int> > queue;
				
				// Add point to hotspots vector
				//TODO: I DONT THINK THIS IS NEEDED!!!
				/*
				hotspots[counter][0] = minPt.z + float(j)/100;
				hotspots[counter][1] = minPt.x + float(i)/100;
				hotspots[counter][2] = matrix[i][j];
				*/

				// Place hotspot in queue
				ROS_DEBUG("Placing hotspot in queue");
				std::vector<int> myvector;
  				myvector.push_back(i);
  				myvector.push_back(j);  		
    			queue.push_back(myvector); //the queue is initialized with the location of a hotspot (?)
    			myvector.clear();

				ROS_DEBUG("Initializing variables");
    			int new_i;
    			int new_j;
    			int loc_x=0;
    			int loc_z=0;
    			int whilecounter = 0;
    			float temp = matrix[i][j];

    			// Start a while-loop clearing elements in a 3x3 grid around the queue elements
    			ROS_DEBUG("Above the while loop");
    			while(queue.size()){
					for(int k=-1; k<2; k++){  //this checks values at locations -1, 0, and 1
						for(int l=-1; l<2; l++){
							new_i = queue[0][0]; //new_i and new_j contain the location of the hotspot
							new_j = queue[0][1];
							if((k+new_i<no_of_rows)&&(k+new_i>=0)&&(l+new_j<no_of_cols)&&(l+new_j>=0)) {  // Check to make sure the square is within the boundary
								if(((k==-1)||(k==1)||(l==-1)||(l==1))&&(matrix[k+new_i][l+new_j]>noise_threshold)){  //RIGHT HERE IS THE NOISE THRESHOLD!! It also skips the hotspot itself
									myvector.push_back(new_i+k); 
									myvector.push_back(new_j+l);  				
					    			queue.push_back(myvector); //here I'm adding other neighbors to the queue
					    			myvector.clear();
					    			loc_x+=(k+new_i); //it appears that I'm summing up all of the point locations
									loc_z+=(l+new_j);
									matrix[k+new_i][l+new_j] = 0;
									whilecounter++; 
								}
								ROS_DEBUG("Inner for - in the while");
								/*
								// Save the largest value found in the region search (temp is the original hotspot location)
								if(matrix[k+new_i][l+new_j] > temp){
									temp = matrix[k+new_i][l+new_j];
									hotspots[counter][0] = minPt.z + float(l+new_j)/100; //here I reassign the hotspot location
									hotspots[counter][1] = minPt.y + float(k+new_i)/100;
									hotspots[counter][2] = matrix[k+new_i][l+new_j];
								}
								
								// Decimate the spot
								if (matrix[k+new_i][l+new_j] > noise_threshold){ //TODO: THIS CHECK ISN"T NEEDED!!!
									loc_x+=(k+new_i); //it appears that I'm summing up all of the point locations
									loc_z+=(l+new_j);
									matrix[k+new_i][l+new_j] = 0;
									whilecounter++; 
								}
								*/

								if(gnu_counter==2){
									std::cout << "loc_x: " << loc_x << ", loc_z: " << loc_z << std::endl;
								}

							}
						}
					}
					queue.erase(queue.begin()); 

					//----------DEBUGGING!---------
					if(gnu_counter==2){
						std::cout << "Elements of queue: " << std::endl;						
						for(int t=0; t<queue.size(); t++){
							for(int u=0; u<queue[t].size(); u++){							
								std::cout << " " << queue[t][u];
							}
							std::cout << std::endl;
						}
					}
					//------------end debugging-----------------
    			}
    			std::cout << "whilecounter is: " << whilecounter << ", loc_x averaged is: " << (float(loc_x)/whilecounter)/100. << ", loc_z: " << (float(loc_z)/whilecounter)/100. << std::endl;
    			hotspots[counter][1] = minPt.x + (float(loc_x)/whilecounter)/100.; //...and then here i'm averaging those locations
    			hotspots[counter][0] = minPt.z + (float(loc_z)/whilecounter)/100.; 
				ROS_INFO_STREAM("Slice " << gnu_counter << ", hotspot loc x: " << (hotspots[counter][1] - minPt.x)*100 << ", z: " << (hotspots[counter][0] - minPt.z)*100);
				hotspots_gnuplot[counter][1] = int(float(loc_x)/whilecounter + 0.5); // the 0.5 is to make the int split the difference in rounding
    			hotspots_gnuplot[counter][0] = int(float(loc_z)/whilecounter + 0.5);
    			//outFile << loc_x/whilecounter << " " << loc_z/whilecounter << " " << 5000 << "\n";
				counter++;
			}
		}
	}


	//--------------_THIS IS JUST FOR DEBUGGING!!-------------
    int flag;
	for(int i=0; i<no_of_rows; i++){
		for(int j=0; j<no_of_cols; j++){
			for(int k=0; k<counter; k++){
				if(i==hotspots_gnuplot[k][1] && j==hotspots_gnuplot[k][0]) {
					flag=1;
					ROS_INFO_STREAM("-----FLAG SET-----, " << hotspots_gnuplot[k][1] << " " << hotspots_gnuplot[k][0]);
				}
			}
			if(flag==1) {
				otherFile << i << " " << j << " " << 5000 << "\n";
			}
			else{
				otherFile << i << " " << j << " " << 0 << "\n";				
			}
			flag = 0;
		}
	}
	//----------------end of debugging--------------

    outFile.close();
    otherFile.close();  //just for debugging!
 	return(hotspots);
}

std::vector<std::vector<float> > Stitch::sphereLogic2(std::vector<std::vector<std::vector<float> > > sphere_locs_3d){

	std::vector<std::vector<float> > sphere_locs (0,std::vector<float>(8, 0));
	std::vector<std::vector<float> > RANSAC_vec (0,std::vector<float>(3, 0));
	std::vector<std::vector<float> > inliers_vec (0,std::vector<float>(3, 0));
	std::vector<std::vector<float> > inliers_temp_vec (0,std::vector<float>(3, 0));
	std::vector<float> distfinal_vec (0, 0);
	std::vector<float> distfinal_temp_vec (0, 0);

	//FILL THE SPHERE_LOCS VECTOR WITH CYLINDER LOCATION PAIRS
	int num_slices = sphere_locs_3d.size();
	int counter = 0;
	float z_point; float y_point; float x_point;

	//iterate through each point in the first layer
	ROS_DEBUG_STREAM("Iterating through " << sphere_locs_3d[0].size() << " spheres in the zeroth layer");
	for(int i=0; i<sphere_locs_3d[0].size(); i++){ 
		if (sphere_locs_3d[0][i][2]!=0){ //if the z_value doesn't equal zero
			x_point = sphere_locs_3d[0][i][0];
			y_point = sphere_locs_3d[0][i][1];
			z_point = sphere_locs_3d[0][i][2];

			// Search for points in other layers within a z-x boundary
			for(int slice=1; slice<(num_slices); slice++){ //iterate through each slice other than the first
				for(int j=0; j<sphere_locs_3d[slice].size(); j++){ //iterate through each point
					if ((std::abs(z_point - sphere_locs_3d[slice][j][2]) < slice*z_threshold)&&(std::abs(x_point - sphere_locs_3d[slice][j][0]) < slice*x_threshold)){ //z and y threshold on connected stalks		
						//Write the points to a vector; 
						RANSAC_vec.push_back(sphere_locs_3d[slice][j]);  //RANSAC_vec is all of the inliers related to a particular base point
					}
				}
			}
			ROS_DEBUG_STREAM("Number of inliers associated with point " << i << " in layer 0: " << RANSAC_vec.size() << " inliers");

			int inliers_temp = 0;
			float dist_sum = 0;
			Vec3<double> C(x_point, y_point, z_point); // Define the base point as a Vec3
			std::vector<float> optimal; //(0.,0.,0.);
			optimal.resize(3,0);

			// Loop through all points in RANSAC_vec, each defining a line
			for(int iterator=0; iterator<RANSAC_vec.size(); iterator++){

				//Define the line between C and B
	    		Vec3<double> B(RANSAC_vec[iterator][0], RANSAC_vec[iterator][1], RANSAC_vec[iterator][2]); // this is the other point defining the line
				int inliers_counter = 0;

				for(int j=0; j<RANSAC_vec.size(); j++){ //Loop through all points in RANSAC_vec, checking the distance from the line

					//Check the distance from the line for each point
					Vec3<double> A (RANSAC_vec[j][0], RANSAC_vec[j][1], RANSAC_vec[j][2]);
					double denominator = pow(Vec3<double>::getDistance(C, B),2);
					double numerator = Vec3<double>::dotProduct((A-C), (B-C));
					Vec3<double> dist_vec = (A-C)-(B-C)*(numerator/denominator);
					double dist_final = sqrt(Vec3<double>::dotProduct(dist_vec, dist_vec));
				    if(dist_final < RANSAC_threshold){
				    	// save inliers here in a vector
				    	std::vector<float> myvector;
		  				myvector.push_back(RANSAC_vec[j][0]);
		  				myvector.push_back(RANSAC_vec[j][1]);
		  				myvector.push_back(RANSAC_vec[j][2]);
						inliers_temp_vec.push_back(myvector);
						distfinal_temp_vec.push_back(dist_final);
						//cout << "Dist final: " << dist_final << endl;
						//cout << RANSAC_vec[j][0] << endl;
						myvector.clear();

				    	inliers_counter++;
				    }
				}
				if(inliers_counter>=inliers_temp){
					inliers_temp = inliers_counter;
					optimal[0] = RANSAC_vec[iterator][0];
					optimal[1] = RANSAC_vec[iterator][1];
					optimal[2] = RANSAC_vec[iterator][2];
					//dist_sum = std::accumulate(distfinal_temp_vec.begin(), distfinal_temp_vec.end(), 0);
					//cout << "DIST SUM IS: " << dist_sum << endl;
					//append vector of inliers
					for(int i=0; i<inliers_temp_vec.size(); i++){
						inliers_vec.push_back(inliers_temp_vec[i]);
						dist_sum+=distfinal_temp_vec[i];
					}
					//cout << "DIST SUM IS: " << dist_sum << endl;
				}
				inliers_temp_vec.clear();
				distfinal_temp_vec.clear();
			}

			// Add the Slice0 point and the optimal point to sphere_locs
			if(inliers_temp>2){ //3
				//int num_inliers = 0;
				std::vector<float> myvector;
  				myvector.push_back(x_point);
  				myvector.push_back(y_point);
  				myvector.push_back(z_point);
  				myvector.push_back(optimal[0]); 
  				myvector.push_back(optimal[1]); 
  				myvector.push_back(optimal[2]); 
  				myvector.push_back(inliers_temp); 
  				myvector.push_back(dist_sum); 
				sphere_locs.push_back(myvector);
				myvector.clear();
			}
			//display_vector(optimal);
			optimal.clear();
			RANSAC_vec.clear();
			counter++;
		}
	}

	//Remove spheres captured from the first row from sphere_locs_3d
	for(int i=0; i<inliers_vec.size(); i++){
		for(int layer=0; layer<sphere_locs_3d.size(); layer++){ //iterate through the six layers
			for(int j=0; j<sphere_locs_3d.size(); j++){ //iterate through the spheres in each layer			
				if((inliers_vec[i][0]==sphere_locs_3d[layer][j][0])&&(inliers_vec[i][1]==sphere_locs_3d[layer][j][1])&&(inliers_vec[i][2]==sphere_locs_3d[layer][j][2])){
					//cout << "in the first" << endl;
					std::cout << "X: " << inliers_vec[i][0] << ", Y:" << inliers_vec[i][1] << ", Z:" << inliers_vec[i][2] << std::endl;
					sphere_locs_3d[layer][j][0] = 0;
					sphere_locs_3d[layer][j][1] = 0;
					sphere_locs_3d[layer][j][2] = 0;
				}
			}
		}
	}

	//iterate through points in the second layer
	ROS_DEBUG_STREAM("Iterating through " << sphere_locs_3d[1].size() << " spheres in the first layer");
	float dist_sum = 0;
	for(int i=0; i<sphere_locs_3d[1].size(); i++){ 
		if (sphere_locs_3d[1][i][2]!=0){
			x_point = sphere_locs_3d[1][i][0];
			y_point = sphere_locs_3d[1][i][1];
			z_point = sphere_locs_3d[1][i][2];

			// Search for points in other layers within a z-x boundary
			for(int slice=2; slice<(num_slices); slice++){ //iterate through each slice other than the first
				for(int j=0; j<sphere_locs_3d[slice].size(); j++){ //iterate through each point
					if ((std::abs(z_point - sphere_locs_3d[slice][j][2]) < slice*z_threshold)&&(std::abs(x_point - sphere_locs_3d[slice][j][0]) < slice*x_threshold)){ //z and y threshold on connected stalks
						
						//Write the points to a vector; 
						RANSAC_vec.push_back(sphere_locs_3d[slice][j]);  //RANSAC_vec is all of the inliers related to a particular base point
					}
				}
			}

			ROS_DEBUG_STREAM("Number of inliers associated with layer 1: " << RANSAC_vec.size());

			int inliers_temp = 0;
			Vec3<double> C(x_point, y_point, z_point); // Define the base point as a Vec3
			std::vector<float> optimal; //(0.,0.,0.);
			optimal.resize(3,0);

			// Loop through all points in RANSAC_vec, each defining a line
			for(int iterator=0; iterator<RANSAC_vec.size(); iterator++){

				//Define the line between C and B
	    		Vec3<double> B(RANSAC_vec[iterator][0], RANSAC_vec[iterator][1], RANSAC_vec[iterator][2]); // this is the other point defining the line
				int inliers_counter = 0;

				for(int j=0; j<RANSAC_vec.size(); j++){ //Loop through all points in RANSAC_vec, checking the distance from the line

					//Check the distance from the line for each point
					Vec3<double> A (RANSAC_vec[j][0], RANSAC_vec[j][1], RANSAC_vec[j][2]);
					double denominator = pow(Vec3<double>::getDistance(C, B),2);
					double numerator = Vec3<double>::dotProduct((A-C), (B-C));
					Vec3<double> dist_vec = (A-C)-(B-C)*(numerator/denominator);
					double dist_final = sqrt(Vec3<double>::dotProduct(dist_vec, dist_vec));
				    //cout << "dist_final: " << dist_final << ", ";
				    if(dist_final < RANSAC_threshold){
				    	//cout << "X-loc: " << RANSAC_vec[j][0] << ", Y-loc: " << RANSAC_vec[j][1] << ", Z-loc: " << RANSAC_vec[j][2] << endl;
				    	distfinal_temp_vec.push_back(dist_final);
				    	inliers_counter++;
				    }
				}
				//cout << "Endfor" << endl;
				if(inliers_counter>=inliers_temp){
					inliers_temp = inliers_counter;
					optimal[0] = RANSAC_vec[iterator][0];
					optimal[1] = RANSAC_vec[iterator][1];
					optimal[2] = RANSAC_vec[iterator][2];
					for(int i=0; i<distfinal_temp_vec.size(); i++){
						dist_sum+=distfinal_temp_vec[i];
					}
				}
				distfinal_temp_vec.clear();
			}
			//cout << "inliers_temp: " << inliers_temp << endl;
			// Add the Slice0 point and the optimal point to sphere_locs
			if(inliers_temp>2){ //3
				//int num_inliers=0;
				std::vector<float> myvector;
  				myvector.push_back(x_point);
  				myvector.push_back(y_point);
  				myvector.push_back(z_point);
  				myvector.push_back(optimal[0]); 
  				myvector.push_back(optimal[1]); 
  				myvector.push_back(optimal[2]); 
  				myvector.push_back(inliers_temp); 
  				myvector.push_back(dist_sum); 
				sphere_locs.push_back(myvector);
				myvector.clear();
			}
			//display_vector(optimal);
			optimal.clear();
			RANSAC_vec.clear();
			counter++;
		}
	}
	return(sphere_locs);
}

void Stitch::slice(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_full, Eigen::Matrix4f GlobalTransform_odom){
	
	pcl::PLYWriter writer;

	ROS_INFO("Calling ArmInitStitch client");
	ros::ServiceClient client = mNode.serviceClient<arm_executive::ArmInitStitch>("arm_init");

	ROS_INFO_STREAM("Size of cloud_full: " << cloud_full->size());
	writer.write ("cloud_full.ply", *cloud_full, false);

	ROS_INFO("Initializing cloud_full_xyz");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_full_xyz (new pcl::PointCloud<pcl::PointXYZ>);	
	copyPointCloud(*cloud_full, *cloud_full_xyz);
	writer.write ("cloud_full_xyz.ply", *cloud_full_xyz, false);

	//--------------------------------Filter Cloud-Full for Rectangular Unreachable Areas---------------
	ROS_DEBUG_STREAM("Size of cloud_full_xyz: " << cloud_full_xyz->size());
	std::clock_t begin = clock();
	pcl::PassThrough<pcl::PointXYZ> pass2;
    pass2.setInputCloud (cloud_full_xyz);
    pass2.setFilterFieldName ("x");
    pass2.setFilterLimits (-arm_extended-cam_x_offset, arm_extended-cam_x_offset-far_reach_reduction); //far_reach_reduction limits reaches over toward the camera
    pass2.filter (*cloud_full_xyz);
    std::cout << "Size of cloud_full_xyz: " << cloud_full_xyz->size() << std::endl;

	pcl::PassThrough<pcl::PointXYZ> pass3;
    pass3.setInputCloud (cloud_full_xyz);
    pass3.setFilterFieldName ("z");
    pass3.setFilterLimits (0, arm_extended+cam_z_offset);
    pass3.filter (*cloud_full_xyz);

    std::cout << "Size of cloud_full_xyz: " << cloud_full_xyz->size() << std::endl;
    std::clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	writer.write ("cloud_full_xyz_dist_filtered.ply", *cloud_full_xyz, false);

	//-----------------------------Voxellize the cloud!!!----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_full_xyz_vox (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_full_xyz);
	sor.setLeafSize(slice_leafsize, slice_leafsize, slice_leafsize);
	sor.filter(*cloud_full_xyz_vox);

	ROS_DEBUG("Initializing to calculate normals");
	//----------------------------------------Take Normals of tformed plants---------------------------------
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::Search<pcl::PointXYZ>::Ptr tree;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
	tree->setInputCloud (cloud_full_xyz_vox);

	writer.write ("cloud_full_xyz_vox.ply", *cloud_full_xyz_vox, false);

	ROS_INFO("Calculating normals");
	ne.setInputCloud (cloud_full_xyz_vox);
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.02); //TODO: perhaps change this to a number of neighbors
	ne.compute (*cloud_normals);

	ROS_DEBUG("Goofy for-loop");
	for(int i=0; i<cloud_normals->size(); i++){
		(*cloud_normals)[i].x = (*cloud_full_xyz_vox)[i].x;
		(*cloud_normals)[i].y = (*cloud_full_xyz_vox)[i].y;
		(*cloud_normals)[i].z = (*cloud_full_xyz_vox)[i].z;;
	}

	ROS_DEBUG("Removing NAN normals");
	std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*cloud_normals,*cloud_normals, indices);

    ROS_DEBUG_STREAM("NaN normals removed, Size of cloud_normals: " << cloud_normals->size());   

	std::vector<std::vector<std::vector<float> > > sphere_locs_3d (num_slices,std::vector<std::vector<float> >(num_centroids_layer,std::vector<float>(3, 0)));  // HARD CODING 25 SPHERES IN ONE LAYER!!!

	// Turn each cluster index into a point cloud
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> multiCloud;

	pcl::PointCloud<MyPointType>::Ptr sphereCloud (new pcl::PointCloud<MyPointType>);
  	sphereCloud->width = num_spheres; //100;
  	sphereCloud->height = 1;
  	sphereCloud->is_dense = false;
	sphereCloud->points.resize (sphereCloud->width * sphereCloud->height);

  	int globalSphereCount = 0;
  	ROS_DEBUG("above slice loop");

    for(int n=0; n<num_slices; n++){
	    //-----------------------------------------Take a 20cm Slice------------------------------------------
	    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_slice (new pcl::PointCloud<pcl::PointNormal>);

	    // Create passthrough filter and slice the cloud
	    pcl::PassThrough<pcl::PointNormal> pass;
	    pass.setInputCloud (cloud_normals);
	    pass.setFilterFieldName ("y");
	    float height_bottom = bottom_start-0.1*(n);
	    float height_middle = middle_start-0.1*(n);
	    float height_top = top_start-0.1*(n);
	    pass.setFilterLimits (height_bottom, height_top);
	    std::cout << "above cloud slice filter" << std::endl;
	    pass.filter (*cloud_slice); 

	    std::stringstream ss8;
	    ss8 << "slice" << n << ".ply";
	    writer.write (ss8.str(), *cloud_slice, false);

		std::vector<int> indices3;
	    pcl::removeNaNFromPointCloud(*cloud_slice,*cloud_slice, indices3);
	
	    //-------------------------------------Heat Map / Filter It-----------------------------------

		std::vector<std::vector<float> > hotspots;

	    // filter one slice of the cloud
	    try{
	    	hotspots = Stitch::hotspotfilterFunction(cloud_slice, n); 
	    }
	    catch(std::exception &e){
	    	ROS_ERROR("Hotspotfilterfxn EXCEPTION");
	    }
	    
		uint32_t shape = visualization_msgs::Marker::SPHERE;
  		visualization_msgs::Marker marker_sphere;
	    std::stringstream ss9;
  		ss9 << "basic_shapes_" << n;
	    
	    //SPHERES UP THE ENTIRE STALK
	    for(int q=0; q<hotspots.size(); q++){
	    	if(hotspots[q][0]!=0){
		    	ROS_DEBUG_STREAM("hotspots x: " << hotspots[q][0]); 
			    ROS_DEBUG_STREAM("hotspots y: " << hotspots[q][1]);

			    sphere_locs_3d[n][q][0] = hotspots[q][1];
	  			sphere_locs_3d[n][q][1] = height_middle;
	  			sphere_locs_3d[n][q][2] = hotspots[q][0];

		    	marker_sphere.header.frame_id = "/multisense/left_camera_optical_frame";
			    marker_sphere.header.stamp = ros::Time::now();
			    marker_sphere.ns = ss9.str();
			    marker_sphere.id = q;
			    ROS_DEBUG_STREAM("Sphere iterator: " << q);
			    marker_sphere.type = shape;
			    marker_sphere.action = visualization_msgs::Marker::ADD;
				marker_sphere.pose.position.x = hotspots[q][1];
	    		marker_sphere.pose.position.y = height_middle;
	    		marker_sphere.pose.position.z = hotspots[q][0];
	    		marker_sphere.pose.orientation.x = 0;
	    		marker_sphere.pose.orientation.y = 0;
	    		marker_sphere.pose.orientation.z = 0;
	    		marker_sphere.pose.orientation.w = 1;
	    		marker_sphere.scale.x = 0.05;
			    marker_sphere.scale.y = 0.05;
			    marker_sphere.scale.z = 0.05;
			    marker_sphere.color.r = 0.0f;
			    marker_sphere.color.g = 1.0f;
			    marker_sphere.color.b = 0.0f;
			    marker_sphere.color.a = 0.2;
			    marker_sphere.lifetime = ros::Duration();
			    markerPub.publish(marker_sphere);  
			}
		} 

	
		//cout << "out of loop assigning points to sphereCloud" << endl;

  		ROS_DEBUG("Above sphere iterator");
  		//std::cout << "Sphere size: " << spheres->points.size() << std::endl;

    	std::stringstream ss;

		// write just the filtered slice (no region growing yet)
	    writer.write ("slice_filtered.ply", *cloud_slice, false);
	}
	ROS_DEBUG("end of slicing loop");

	//----------------------------BEGINNING OF CYLINDER DETECTION------------------------------
    	
    
	std::vector<std::vector<float> > cylinder_locs;

	cylinder_locs = Stitch::sphereLogic2(sphere_locs_3d); //sphere_locs is an n x 8 vector, where n is the number of cylinders


	//shape = visualization_msgs::Marker::SPHERE;
	visualization_msgs::Marker marker_sphere;
    std::stringstream ss10;
    
    //SPHERE AT BOTTOM OF STALK
    for(int q=0; q<cylinder_locs.size(); q++){
    	ss10 << "basic_shapes2_" << q;
    	marker_sphere.header.frame_id = "/multisense/left_camera_optical_frame";
	    marker_sphere.header.stamp = ros::Time::now();
	    marker_sphere.ns = ss10.str();
	    marker_sphere.id = q;
	    ROS_DEBUG_STREAM("Lower sphere: " << cylinder_locs[q][0] << ", " << cylinder_locs[q][1] << ", " << cylinder_locs[q][2]);
	    ROS_DEBUG_STREAM("Upper sphere: " << cylinder_locs[q][3] << ", " << cylinder_locs[q][4] << ", " << cylinder_locs[q][5]);	    
	    marker_sphere.type = visualization_msgs::Marker::SPHERE;
	    marker_sphere.action = visualization_msgs::Marker::ADD;
		marker_sphere.pose.position.x = cylinder_locs[q][0];
		marker_sphere.pose.position.y = cylinder_locs[q][1];
		marker_sphere.pose.position.z = cylinder_locs[q][2];
		marker_sphere.pose.orientation.x = 0;
		marker_sphere.pose.orientation.y = 0;
		marker_sphere.pose.orientation.z = 0;
		marker_sphere.pose.orientation.w = 1;
		marker_sphere.scale.x = 0.05;
	    marker_sphere.scale.y = 0.05;
	    marker_sphere.scale.z = 0.05;
	    marker_sphere.color.r = 0.0f;
	    marker_sphere.color.g = 0.0f;
	    marker_sphere.color.b = 1.0f;
	    marker_sphere.color.a = 0.5;
	    marker_sphere.lifetime = ros::Duration();
	    usleep(1000);
	    markerPub.publish(marker_sphere);  
	} 	 

    std::stringstream ss11;
	visualization_msgs::Marker marker_sphere2;
	//SPHERE AT TOP OF STALK
    for(int q=0; q<cylinder_locs.size(); q++){
    	ss11 << "basic_shapes3_" << q;
    	marker_sphere2.header.frame_id = "/multisense/left_camera_optical_frame";
	    marker_sphere2.header.stamp = ros::Time::now();
	    marker_sphere2.ns = ss11.str();
	    marker_sphere2.id = q;
	    ROS_DEBUG_STREAM("Sphere iterator: " << q);
	    marker_sphere2.type = visualization_msgs::Marker::SPHERE;
	    marker_sphere2.action = visualization_msgs::Marker::ADD;
		marker_sphere2.pose.position.x = cylinder_locs[q][3];
		marker_sphere2.pose.position.y = cylinder_locs[q][4];
		marker_sphere2.pose.position.z = cylinder_locs[q][5];
		marker_sphere2.pose.orientation.x = 0;
		marker_sphere2.pose.orientation.y = 0;
		marker_sphere2.pose.orientation.z = 0;
		marker_sphere2.pose.orientation.w = 1;
		marker_sphere2.scale.x = 0.05;
	    marker_sphere2.scale.y = 0.05;
	    marker_sphere2.scale.z = 0.05;
	    marker_sphere2.color.r = 1.0f;
	    marker_sphere2.color.g = 0.0f;
	    marker_sphere2.color.b = 0.0f;
	    marker_sphere2.color.a = 1.0;
	    marker_sphere2.lifetime = ros::Duration();
	    usleep(1000);
	    markerPub.publish(marker_sphere2);  
	} 


	float dist;
	std::vector<int> to_remove;

	std::cout << "Number of cylinders: " << cylinder_locs.size() << std::endl; 

	for(int i=0; i<cylinder_locs.size(); i++){ //i defines the line
		for(int j=0; j<cylinder_locs.size(); j++){  //j defines the top point of the other lines
			if(i!=j){
				//check distance from each top point to each line segment
				Vec3<double> X2 (cylinder_locs[i][0], cylinder_locs[i][1], cylinder_locs[i][2]);
				Vec3<double> X1 (cylinder_locs[i][3], cylinder_locs[i][4], cylinder_locs[i][5]);
				Vec3<double> X0 (cylinder_locs[j][3], cylinder_locs[j][4], cylinder_locs[j][5]);

				float num1 = pow(Vec3<double>::getDistance(X1, X0),2);
				float num2 = pow(Vec3<double>::getDistance(X2, X1),2);
				float num3 = pow(Vec3<double>::dotProduct(X1-X0, X2-X1),2);
				float denom = num2;
				float dist = ((num1*num2)-num3)/denom;
				//cout << "Distance: " << dist << endl;
				if(dist < cylinder_dist_threshold){
					if(cylinder_locs[i][6]>cylinder_locs[j][6]){ // check if the distance is below the threshold and compare number of points captured
						to_remove.push_back(j);
						//cout << "Weight: " << cylinder_locs[i][6] << ", Weight: " << cylinder_locs[j][6] << endl;
					}
					else if(cylinder_locs[i][6]<cylinder_locs[j][6]) {
						to_remove.push_back(i);
					}
					else{
						std::cout << "------------THEY'RE EQUAL!------------" << std::endl;
						std::cout << "dist_sum: " << cylinder_locs[i][7] << ", " << cylinder_locs[j][7] << std::endl;
						if(cylinder_locs[i][7]>cylinder_locs[j][7]){ //if point i has a larger error than point j
							to_remove.push_back(i);
						}
						else{
							to_remove.push_back(j);
						}
					}
				}
			}
		}
	}

	//cout << "to remove: " << to_remove << endl;
	for(int i=0; i<to_remove.size(); i++){
		std::cout << "To REMOVE: " << to_remove[i] << std::endl;
	}
  	
  	uint32_t shape = visualization_msgs::Marker::CYLINDER;
  	visualization_msgs::Marker marker;
  	
  	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize (7);    // We need 7 values
  	for(int i=0; i<cylinder_locs.size(); i++){
  		//if(i == std::end(to_remove)){ //check if 'i' is one of the cylinders we want to remove
  		if(std::find(to_remove.begin(), to_remove.end(), i) == to_remove.end()) {  //check if 'i' is one of the cylinders we want to remove
	  		std::stringstream ss6;
	  		ss6 << "cylinder" << i;

		    marker.header.frame_id = "/multisense/left_camera_optical_frame";
		    marker.header.stamp = ros::Time::now();
		    marker.ns = "basic_shapes";
		    marker.id = i;
		    std::cout << "Cylinder iterator: " << i << std::endl;
		    marker.type = shape;
		    marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = (cylinder_locs[i][0]+cylinder_locs[i][3])/2;
    		marker.pose.position.y = (cylinder_locs[i][1]+cylinder_locs[i][4])/2;
    		marker.pose.position.z = (cylinder_locs[i][2]+cylinder_locs[i][5])/2;
    		double x_orientation = cylinder_locs[i][3] - cylinder_locs[i][0];
    		double y_orientation = cylinder_locs[i][4] - cylinder_locs[i][1];
    		double z_orientation = cylinder_locs[i][5] - cylinder_locs[i][2];
    		ROS_DEBUG_STREAM("----------------VECTOR: " << x_orientation << ", " << y_orientation << ", " << z_orientation);
			Eigen::Vector3d v1(x_orientation,y_orientation,z_orientation);
			Eigen::Vector3d v2(0,0,1);
			Eigen::Vector3d a = v2.cross(v1);
		    ROS_DEBUG_STREAM("------------a is: " << a[0] << ", " << a[1] << ", " << a[2]);
			double w = sqrt(pow(v1.norm(),2) * pow(v2.norm(),2)) + v2.dot(v1);
			double quat_norm = sqrt(pow(a[0],2)+pow(a[1],2)+pow(a[2],2)+pow(w,2));
			//std::cout << "Quaternion Norm: " << quat_norm << std::endl;
    		//std::cout << a[x] << std::endl;
    		ROS_DEBUG_STREAM("----------------QUATERNION: " << a[0]/quat_norm << ", " << a[1]/quat_norm << ", " << a[2]/quat_norm << ", " << w/quat_norm);

    		marker.pose.orientation.x = a[0]/quat_norm;
    		marker.pose.orientation.y = a[1]/quat_norm;
    		marker.pose.orientation.z = a[2]/quat_norm;
    		marker.pose.orientation.w = w/quat_norm;
    		marker.scale.x = 0.05;
		    marker.scale.y = 0.05;
		    marker.scale.z = 0.8;
		    marker.color.r = 0.0f;
		    marker.color.g = 1.0f;
		    marker.color.b = 0.0f;
		    marker.color.a = 0.3;
		    marker.lifetime = ros::Duration();
		    markerPub.publish(marker);

		    //CALCULATE CYLINDER CENTROID AT CAMERA HEIGHT
		    //-----Interpolates between top and bottom points at gripper height------

		    double height_ratio = (-arm_height-cylinder_locs[i][1])/(cylinder_locs[i][4]-cylinder_locs[i][1]);  //arm_height is negative because TF from camera to stage is flipped
		    double x_loc_camera_frame = cylinder_locs[i][0]+(height_ratio)*(cylinder_locs[i][3]-cylinder_locs[i][0]);
			double z_loc_camera_frame = cylinder_locs[i][2]+(height_ratio)*(cylinder_locs[i][5]-cylinder_locs[i][2]);	

			ROS_DEBUG_STREAM("-------------Height ratio: " << height_ratio << ", x_loc: " << x_loc_camera_frame << ", z_loc: " << z_loc_camera_frame);
			
			//SPHERE AT REACH POINT
			visualization_msgs::Marker marker_sphere4;
	    	marker_sphere4.header.frame_id = "/multisense/left_camera_optical_frame";
		    marker_sphere4.header.stamp = ros::Time::now();
		    marker_sphere4.ns = "basic_shapes4";
		    marker_sphere4.id = i;
		    marker_sphere4.type = visualization_msgs::Marker::SPHERE;
		    marker_sphere4.action = visualization_msgs::Marker::ADD;
			marker_sphere4.pose.position.x = x_loc_camera_frame;
			marker_sphere4.pose.position.y = -arm_height;
			marker_sphere4.pose.position.z = z_loc_camera_frame;
			marker_sphere4.pose.orientation.x = 0;
			marker_sphere4.pose.orientation.y = 0;
			marker_sphere4.pose.orientation.z = 0;
			marker_sphere4.pose.orientation.w = 1;
			marker_sphere4.scale.x = 0.05;
		    marker_sphere4.scale.y = 0.05;
		    marker_sphere4.scale.z = 0.05;
		    marker_sphere4.color.r = 0.5f;
		    marker_sphere4.color.g = 0.5f;
		    marker_sphere4.color.b = 0.0f;
		    marker_sphere4.color.a = 1.0;
		    marker_sphere4.lifetime = ros::Duration();
		    usleep(1000);
		    markerPub.publish(marker_sphere4);  
			

		    //-----------Check if yellow sphere is reachable!!----------------

		    centroid_x = -(x_loc_camera_frame + cam_x_offset); //centroids in arm_link coordinate frame
		    centroid_y = -(z_loc_camera_frame + cam_z_offset); //TODO: MAKE THIS MAKE SENSE	

		    ROS_INFO_STREAM("centroid_x: " << centroid_x);
		    ROS_INFO_STREAM("centroid_y: " << centroid_y);

		    if(sqrt(pow(centroid_x,2)+pow(centroid_y,2))<arm_extended){

			    std::vector<float> centroids_temp;
			    centroids_temp.push_back(centroid_x);	
			    centroids_temp.push_back(centroid_y);
			    centroids.push_back(centroids_temp);
			    centroids_temp.clear();	

				arm_executive::ArmInitStitch srv;
			  	srv.request.arm_init_req = "left";
			  	srv.request.arm_init_req_x = centroid_x;
			  	srv.request.arm_init_req_y = centroid_y;

			  	if (client.call(srv))
			  	{
			    	ROS_INFO_STREAM("Response: " << srv.response.arm_init_resp);
			  	}
			  	else
			  	{
			    	ROS_ERROR("Failed to call service");
			  	}
			}
		    else{
		  	  ROS_ERROR("Centroid too far away!");
		    }		    		        
		}
	}
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "stitcher_server");

  	ros::NodeHandle nh;

	Stitch* pStitch = new Stitch;

	log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);


  	message_filters::Subscriber<nav_msgs::Odometry> odomSub(nh, "/wheel_encoder/odom", 20);
  	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub(nh, "/multisense/image_points2_color", 20);
  	message_filters::Subscriber<sensor_msgs::JointState> jointSub(nh, "/joint_states", 20);

  	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2, sensor_msgs::JointState> MySyncPolicy;

  	Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), odomSub, cloudSub, jointSub);
  	sync.registerCallback(boost::bind(&Stitch::callback, pStitch, _1, _2, _3));

	ros::spin();
	
	return 0;
}
