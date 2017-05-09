#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
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

using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

class Stitch
{
public:
	Stitch();
	//void calcError(const boost::shared_ptr<const PointStamped>& cameraPointPtr, const boost::shared_ptr<const PointStamped>& boardPointPtr, const boost::shared_ptr<const JointState>& jointStatePtr);
	void calcError(const boost::shared_ptr<const nav_msgs::Odometry>& cameraPointPtr, const boost::shared_ptr<const sensor_msgs::PointCloud2>& boardPointPtr);
	
private:
	ros::NodeHandle mNode;
	ros::Publisher mErrorPub;
	ros::Publisher mCommandPub;
	ros::Publisher mArmData;

	float mPError1, mPError2, mIError1, mIError2;
};

Stitch::Stitch()
{
	int buffer = 10;
	mErrorPub = mNode.advertise<std_msgs::String>("chatter", buffer);
}

//void Stitch::calcError(const boost::shared_ptr<const PointStamped>& cameraPointPtr, const boost::shared_ptr<const PointStamped>& boardPointPtr, const boost::shared_ptr<const JointState>& jointStatePtr)
void Stitch::calcError(const boost::shared_ptr<const nav_msgs::Odometry>& cameraPointPtr, const boost::shared_ptr<const sensor_msgs::PointCloud2>& boardPointPtr)
{
	std::cout << "i'm in here!!" << std::endl;
	std_msgs::String msg;
	std::stringstream ss;
	ss << "hello world";
	msg.data = ss.str();
	mErrorPub.publish(msg);
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "kdc_servo");

  	ros::NodeHandle nh;

	Stitch* pStitch = new Stitch();

  	message_filters::Subscriber<nav_msgs::Odometry> cameraPointSub(nh, "/wheel_encoder/odom", 10);
  	message_filters::Subscriber<sensor_msgs::PointCloud2> boardPointSub(nh, "/multisense/image_points2_color", 10);

  	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
  	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cameraPointSub, boardPointSub);
  	sync.registerCallback(boost::bind(&Stitch::calcError, pStitch, _1, _2));

	/*
	ros::init(argc, argv, "kdc_servo");

	std::string cameraTopic;
	std::string boardTopic;
	std::string jointTopic;
	
	ros::NodeHandle nh("~");

	nh.getParam("/odometry/filtered_imu_encoders", cameraTopic);
	nh.getParam("/multisense/image_points2_color", boardTopic);

	Stitch* pStitch = new Stitch();

	int buffer = 1;
	message_filters::Subscriber<nav_msgs::Odometry> cameraPointSub(nh, cameraTopic, buffer);
	message_filters::Subscriber<sensor_msgs::PointCloud2> boardPointSub(nh, boardTopic, buffer);

	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> StitchSync;
	Synchronizer<StitchSync> sync(StitchSync(2), cameraPointSub, boardPointSub);
	
	sync.registerCallback(boost::bind(&Stitch::calcError, pStitch, _1, _2));
	*/

	/*
	nh.getParam("cameraTopic", cameraTopic);
	nh.getParam("boardTopic", boardTopic);
	nh.getParam("jointTopic", jointTopic);

	Stitch* pStitch = new Stitch();

	int buffer = 1;
	message_filters::Subscriber<PointStamped> cameraPointSub(nh, cameraTopic, buffer);
	message_filters::Subscriber<PointStamped> boardPointSub(nh, boardTopic, buffer);
	message_filters::Subscriber<JointState> joint_statesSub(nh, jointTopic, buffer);

	typedef sync_policies::ApproximateTime<PointStamped, PointStamped, JointState> StitchSync;
	Synchronizer<StitchSync> sync(StitchSync(3), cameraPointSub, boardPointSub, joint_statesSub);
	
	// typedef void (VisServo::*OverloadError)(const PointStamped&, const PointStamped&);
	sync.registerCallback(boost::bind(&Stitch::calcError, pStitch, _1, _2, _3));
	*/

	ros::spin();
	return 0;
}
