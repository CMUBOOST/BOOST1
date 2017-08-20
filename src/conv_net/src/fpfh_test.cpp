#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <limits>
#include <pcl/point_types_conversion.h>

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

#include <string.h>
#include <unistd.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <exception>

//#include <point_types_conversion.h>
double radius = 0.01;

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv (new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv_filtered (new pcl::PointCloud<pcl::PointXYZHSV>); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);   
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_filtered_outlier (new pcl::PointCloud<pcl::PointXYZRGB>);      
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

  // Fill in the cloud data
  pcl::PLYReader reader;
  reader.read ("StitchedCloud_new.ply", *cloud);

  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.003f, 0.003f, 0.003f);
  sor.filter (*cloud_filtered);

  /*
  pcl::PointCloudXYZRGBtoXYZHSV(*cloud_filtered, *cloud_hsv);
  int hMax = 41;
  int hMin = 30;

  for(int i=0; i<cloud_hsv->size(); i++){
    if((*cloud_hsv)[i].h < 60 || (*cloud_hsv)[i].h > 82){
     
    }
  }
  
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_hsv,*cloud_hsv, indices);
  pcl::PointCloudXYZHSVtoXYZRGB(*cloud_hsv, *cloud_rgb);
  */
  std::clock_t begin = clock();

  int rMax = 255;
  int rMin = 36;
  int gMax = 255;
  int gMin = 45;
  int bMax = 205;
  int bMin = 26;
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin)));

  // build the filter
  //pcl::ConditionalRemoval<pcl::PointXYZHSV> condrem (color_cond);
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
  condrem.setInputCloud (cloud_filtered);
  condrem.setKeepOrganized(true);
    
  // apply filter
  condrem.filter (*cloud_rgb_filtered);
  
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_rgb_filtered,*cloud_rgb_filtered, indices);

  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
  // build the filter
  outrem.setInputCloud(cloud_rgb_filtered);
  outrem.setRadiusSearch(0.005);
  outrem.setMinNeighborsInRadius (5);
  // apply filter
  outrem.filter (*cloud_rgb_filtered_outlier);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (cloud_rgb_filtered_outlier);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (radius);

  // Compute the features
  ne.compute (*normals);

  /*
  for(int i=0; i<normals->size(); i++){
    //std::cout << (*normals)[i] << std::endl;
    if((*normals)[i].normal_y>0.1 || (*normals)[i].normal_y<-0.1){
      cloud_rgb_filtered_outlier->erase (cloud_rgb_filtered_outlier->begin()+i);
    }
  }

  std::vector<int> indices2;
  pcl::removeNaNFromPointCloud(*cloud_rgb_filtered_outlier,*cloud_rgb_filtered_outlier, indices2);
  */

  /*
  // Create the FPFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (cloud_rgb_filtered_outlier);
  fpfh.setInputNormals (normals);
  // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  //pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);

  fpfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh.setRadiusSearch (radius);

  // Compute the features
  fpfh.compute (*fpfhs);

  // fpfhs->points.size () should have the same size as the input cloud->points.size ()*
  */

  std::clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << elapsed_secs << std::endl;

  pcl::PLYWriter writer;
  writer.write<pcl::PointXYZRGB> ("test.ply", *cloud_rgb_filtered_outlier, false);
}