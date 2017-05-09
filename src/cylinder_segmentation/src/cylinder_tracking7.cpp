#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/distances.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <log4cxx/logger.h>

#include "../../hebi/src/servoVars.hpp"

typedef pcl::PointXYZ PointT;

class CylinderSegmentation
{
    ros::NodeHandle nh;
    ros::Subscriber sub_pointcloud;
    ros::Publisher pub_pointcloud, pub_pointcloud_cylinder, pub_pointcloud_centroid;

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;

    public:
        CylinderSegmentation()
        {
            sub_pointcloud = nh.subscribe("/duo3d_camera/points2",1,&CylinderSegmentation::pointCloudCb, this);
            pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/duo3d_camera/points_filtered", 1);
            pub_pointcloud_cylinder = nh.advertise<sensor_msgs::PointCloud2>("/duo3d_camera/points_cylinder", 1);
            pub_pointcloud_centroid = nh.advertise<geometry_msgs::PointStamped>("/duo3d_camera/points_centroid", 1);
        }

        ~CylinderSegmentation()
        {}

        void extractcylinder(pcl::PointCloud<PointT>::Ptr cloud_filtered)
        {
            Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);

            pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>);

            pcl::NormalEstimation<PointT, pcl::Normal> ne;
            pcl::ExtractIndices<PointT> extract;
            pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices ());
            pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
            pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

            ne.setSearchMethod (tree);
            ne.setInputCloud (cloud_filtered);
            ne.setKSearch (10);
            ne.compute (*cloud_normals);

            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_CYLINDER);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setAxis(axis);
            seg.setEpsAngle(5.0f * (M_PI/180.0f));
            seg.setNormalDistanceWeight (0);
            seg.setMaxIterations (ransacIterations);
            //seg.setDistanceThreshold (.015);
            seg.setDistanceThreshold (distThresh);
            seg.setRadiusLimits (minRadiusLim, maxRadiusLim);
            //seg.setDistanceThreshold (15);
            //seg.setRadiusLimits (10, 40);
            seg.setInputCloud (cloud_filtered);
            seg.setInputNormals (cloud_normals);
            seg.segment (*inliers_cylinder, *coefficients_cylinder);

            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers_cylinder);
            extract.setNegative (false);
            extract.filter (*cloud_cylinder);

            //pcl::PointIndices inliers;
            //pcl::PointXYZ pmin, pmax;
            //float maxdist = pcl::getMaxSegment (*cloud_cylinder, pmin, pmax);

            PointT min_pt, max_pt;
            pcl::getMinMax3D(*cloud_cylinder, min_pt, max_pt);

            //ROS_INFO_STREAM("Max Z Distance: " << max_pt.z-min_pt.z);
            //ROS_INFO_STREAM("Max X Distance: " << max_pt.x-min_pt.x);
            //ROS_INFO_STREAM("Max Y Distance: " << max_pt.y-min_pt.y);

            Eigen::Vector4f centroid;

            int numpoints_inliers = inliers_cylinder->indices.size();
            ROS_DEBUG_STREAM("inliers_cylinder is: " << numpoints_inliers);

            int numpoints = cloud_cylinder->points.size();
            ROS_DEBUG_STREAM("cloud_cylinder is: " << numpoints);

            if ((cloud_cylinder->points.size ()) < 50)  
                ROS_DEBUG_STREAM("Too few points: "  << numpoints);
            else if ((max_pt.y - min_pt.y) < 0.100)
                ROS_DEBUG_STREAM("Points are too close together in Z: " << max_pt.y - min_pt.y);
            else if ((max_pt.x - min_pt.x) < 0.020)
                ROS_DEBUG_STREAM("Points are too close together in X: " << max_pt.x - min_pt.x);
            else
            {
                pcl::compute3DCentroid (*cloud_cylinder, centroid);
                //std::cerr << centroid[0] << std::endl << centroid[1] << std::endl << centroid[2] << std::endl;
                ROS_DEBUG_STREAM("------------------Centroid is: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << "---------------");
                //std::cerr << "max distance between points is:" << maxdist << std::endl;

                sensor_msgs::PointCloud2 msg_pub1;
	 	
                geometry_msgs::PointStamped center;

                pcl::toROSMsg(*cloud_cylinder, msg_pub1);

                center.point.x = centroid[0];
                center.point.y = centroid[1];
                center.point.z = centroid[2];

                msg_pub1.header.frame_id = "/point_cloud_link";

                pub_pointcloud_cylinder.publish(msg_pub1);
                pub_pointcloud_centroid.publish(center);
            }

        }

        void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& msg_sub)
        {
            pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
            pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
            pcl::fromROSMsg(*msg_sub, *cloud_in);

            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

            int numpoints_original = cloud_in->points.size();
            ROS_DEBUG_STREAM("Original: "  << numpoints_original);

            //*********PASSTHROUGH FILTER***************//
            // Create the filtering object
            pcl::PassThrough<PointT> pass;
            pass.setInputCloud (cloud_in);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.0, zCameraFilter);
            pass.filter (*cloud_filtered);
            int numpoints_passthrough = cloud_filtered->points.size();
            ROS_DEBUG_STREAM("Passthrough: "  << numpoints_passthrough);

            //*********PASSTHROUGH FILTER***************//
            // Create the filtering object
            pcl::PassThrough<PointT> pass_x;
            pass_x.setInputCloud (cloud_filtered);
            pass_x.setFilterFieldName ("x");
            pass_x.setFilterLimits (xCameraFilterMin, xCameraFilterMax);
            pass_x.filter (*cloud_filtered);
            int numpoints_passthrough_x = cloud_filtered->points.size();
            ROS_DEBUG_STREAM("Passthrough: "  << numpoints_passthrough_x);

            //*********VOXELGRID FILTER**************//
            const float voxel_grid_size = 0.002f;
            pcl::VoxelGrid<PointT> vox_grid;
            vox_grid.setInputCloud (cloud_filtered);
            vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
            //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
            pcl::PointCloud<PointT>::Ptr tempCloud (new pcl::PointCloud<PointT>);
            vox_grid.filter (*tempCloud);
            cloud_filtered = tempCloud;

            int numpoints_voxel = cloud_filtered->points.size();
            ROS_DEBUG_STREAM("Voxel: "  << numpoints_voxel);

            extractcylinder(cloud_filtered);

            sensor_msgs::PointCloud2 msg_pub;
            pcl::toROSMsg(*cloud_filtered, msg_pub);
            msg_pub.header.frame_id = "/point_cloud_link";

            pub_pointcloud.publish(msg_pub);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cylinder_segmentation");
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
    CylinderSegmentation cylSeg;
    ros::spin();
    return 0;
}
