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
#include "Vec3.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <exception>
#include <pcl/features/principal_curvatures.h>

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
namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

typedef boost::circular_buffer<sensor_msgs::PointCloud2> circular_buffer;
typedef boost::circular_buffer<nav_msgs::Odometry> circular_buffer_odom;
static const std::string OPENCV_WINDOW = "Image window";

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
	void callback (const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudPtr, const boost::shared_ptr<const sensor_msgs::Image>& imagePtr);

// ---	
private:
	ros::NodeHandle mNode;
	ros::Publisher cloudPub;
	ros::Publisher markerPub;
	ros::Publisher zPub;
	circular_buffer cb;
	circular_buffer_odom cb_odom;
	Eigen::Matrix4f GlobalTransform, pairTransform;
	Eigen::Matrix4f GlobalTransform_odom;	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_full;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_old;

	const static bool cloudWrite = true;

	int total_green;
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

	const static float offset_HACK = 0.0;

	//--------Stitching Variables-----------
	const static float pairAlign_leafsize = 0.015; //0.01
	const static float pairAlign_tform_eps = 1e-5; //1e-5
	const static float pairAlign_correspondenceDistance = 0.05;
	const static float pairAlign_correspondenceDistance_cutoff = 0.01; //0.01  //raising this value makes the stitching run marginally faster

	//--------RGB Filtering Variables-------- 	
	const static int rMax = 255;
	const static int rMin = 15; //68; //99; //15; //31;
	const static int gMax = 255;
	const static int gMin = 15; //78; //115; //15; //22;
	const static int bMax = 255;
	const static int bMin = 15; //44; //56; //15; //22;

	//----------Heat Map Filtering----------
	//const static int noise_threshold = 400;//40;
	//const static int hotspot_threshold = 800;
	int noise_threshold;
	int hotspot_threshold;

	//---------Sphere/Cylinder Logic--------
	const static float z_threshold = 0.015; //0.02 //this determines the angle of the stalk
	const static float x_threshold = 0.015; //0.02 //this determines the angle of the stalk
	const static float RANSAC_threshold = 0.025;

	//------------Cloud Slicing------------
	const static int rMin_slice = 68; //99; 
	const static int gMin_slice = 78; //115;
	const static int bMin_slice = 44; //56;
	const static int rMax_slice = 250; //99; 
	const static int gMax_slice = 250; //115;
	const static int bMax_slice = 200; //56;	
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
	this->cloudPub = mNode.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("detected_cloud", 1);	

	cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_filtered_old.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_full.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void Stitch::callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudPtr, const boost::shared_ptr<const sensor_msgs::Image>& imagePtr)
{
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_a (new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::fromROSMsg(*cloudPtr, *cloud_a);	
	
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloudPtr,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_a (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_a);

	//pcl::PCLPointCloud2 pcl_pc;
	//pcl_conversions::toPCL(input, pcl_pc);
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	//pcl::fromPCLPointCloud2(pcl_pc, cloud);

	std::cout << "Width: " << cloud_a->width << std::endl;
	std::cout << "Height: " << cloud_a->height << std::endl;	
	std::cout << "Size: " << cloud_a->points.size() << std::endl;

	//cout << cloudPtr->width << endl;
	//cout << cloudPtr->height << endl;
	//cout << cloudPtr->row_step << endl;
	//cout << cloudPtr->data[10000] << endl;	

	
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

	cv::Mat output;

	//cv::inRange(cv_ptr->image, cv::Scalar(36, 45, 26), cv::Scalar(255, 255, 255), output);
	cv::inRange(cv_ptr->image, cv::Scalar(5, 5, 70), cv::Scalar(255, 255, 255), output);
	cv::imshow(OPENCV_WINDOW, output);
    cv::waitKey(3);

    //std::cout << output.size() << std::endl;

    pcl::PLYWriter writer;
	//writer.write<pcl::PointXYZRGB> ("cloud_a", *cloud_a, false);

	//for(int i=0; i<cloud_a->points.size(); i++){    
	//	cloud_a->points[i].r = 255;		
	//}
	cout << "Cols: " << output.cols << endl;
	cout << "Rows: " << output.rows << endl;
	int counter = 0;
	int black_counter = 0;
	for(int y=0; y<output.cols; y++){ //loop through cols?
    	for(int x=0; x<output.rows; x++){ //loop through rows?
    		//Vec3b intensity = output.at<Vec3b>(j,i);
    		int intensity = output.at<uchar>(y,x);
    		if(intensity != 0){
    			counter++;
    		}
    		if(intensity==0)
    			black_counter++;
    	}
    }	

    cout << "Picture counter: " << counter << endl;
    cout << "Picture black counter: " << black_counter << endl;

    
	counter = 0;
    for(int y=0; y<output.cols; y++){ //loop through cols?
    	for(int x=0; x<output.rows; x++){ //loop through rows?
    		//Vec3b intensity2= output.at<Vec3b>(j,i);
    		//if(intensity2[0] == 255){
    		int intensity = output.at<uchar>(y, x);
    		if(intensity != 0){
    			//cout << i << ", " << j << endl;   			
    			//if(!isnan((*cloud_a).points[y*x].x)){
					//cout << (*cloud_a).points[j*i].x << endl;
					//cout << (*cloud_a).points[j*i].g << endl;					
					//ROS_INFO_STREAM(cloud_a->points[j*i]);					
					//cout << cloud_a->points[j*i] << endl;
					cloud_a->points[y*1024+x].r = 255;
					cloud_a->points[y*1024+x].g = 0;
					cloud_a->points[y*1024+x].b = 0;	
					counter++;		
					//cout << (*cloud_a).points[j*i].x << endl;							
					// NOTE: cloud_a->points[j*i] >> r;
					// cout << "jim test" << endl;
					// typedef Point pcl::PointCloud<pcl::PointXYZRGB>::Ptr;
					// Point &p = (*cloud_a).points[j*i];
					// uint32_t rgb = cloud_a->points[j*i].rgb;
  					// uint8_t r = (rgb >> 16) & 0x0000ff;
					// uint8_t g = (rgb >> 8)  & 0x0000ff;
					// uint8_t b = (rgb)       & 0x0000ff;
					// cout << "(r, g, b) = (" << r << ", " << g << ", " << b << ")" << endl;
					//(*cloud_a).points[j*i].r = 'r';
    			//}
    			
    			//cout << (*cloud_a).points[j*i].x << endl;
    		}
    	}
    }

    cout << "Counter: " << counter << endl;

   	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud_a,*cloud_a, indices);
   	writer.write<pcl::PointXYZRGB> ("cloud_a.ply", *cloud_a, false); 

   	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
   	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, 250)));
    //color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, 2)));
    //color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, 2)));

    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
    condrem.setInputCloud (cloud_a);
    condrem.setKeepOrganized(true);
      
    // apply filter
    condrem.filter (*cloud_filtered);
	ROS_DEBUG_STREAM("Cloud_Filtered size:" << cloud_filtered->size());

    std::vector<int> indices2;
    pcl::removeNaNFromPointCloud(*cloud_filtered,*cloud_filtered, indices2);
    writer.write<pcl::PointXYZRGB> ("cloud_a_filtered.ply", *cloud_filtered, false); 
	
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "cloud_detect_server");

  	ros::NodeHandle nh;

	Stitch* pStitch = new Stitch;

	log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

  	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub(nh, "/multisense/organized_image_points2_color", 20);
  	message_filters::Subscriber<sensor_msgs::Image> imageSub(nh, "/multisense/left/image_rect_color", 20);  	

  	typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

  	Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), cloudSub, imageSub);
  	sync.registerCallback(boost::bind(&Stitch::callback, pStitch, _1, _2));

	ros::spin();
	
	return 0;
}
