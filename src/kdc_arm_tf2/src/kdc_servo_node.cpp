#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <string>
#include <cmath>

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

using namespace message_filters;
using namespace geometry_msgs;

class VisServo
{
public:
	VisServo();
	void calcError(const boost::shared_ptr<const PointStamped>& cameraPoint, const boost::shared_ptr<const PointStamped>& boardPoint);
	void sendCommand(const PointStamped error);

private:
	ros::NodeHandle mNode;
	ros::Publisher mErrorPub;
	ros::Publisher mCommandPub;
	
	hebi::Group* mGroup_g;
	hebi::GroupCommand mCmd(3);
};

VisServo::VisServo()
{
	int buffer = 1;
	mErrorPub = mNode.advertise<PointStamped>("kdc_error_topic", buffer);
	mCommandPub = mNode.advertise<sensor_msgs::JointState>("kdc_command_topic", buffer);

	mGroup_g = NULL;
}

void VisServo::calcError(const boost::shared_ptr<const PointStamped>& cameraPoint, const boost::shared_ptr<const PointStamped>& boardPoint)
{
	PointStamped camera = *cameraPoint;
	PointStamped board = *boardPoint;

	PointStamped error;
	error.header = std_msgs::Header();
	error.point.x = camera.point.x - board.point.x;
	error.point.y = camera.point.y - board.point.y;
	error.point.z = camera.point.z - board.point.z;
	
	ROS_INFO("***********************************************************************\n");
	ROS_INFO("Camera Point: (%f, %f, %f)\n", camera.point.x, camera.point.y, camera.point.z);
	ROS_INFO("Board Point: (%f, %f, %f)\n", board.point.x, board.point.y, board.point.z);
	ROS_INFO("Error Point: (%f, %f, %f)\n", error.point.x, error.point.y, error.point.z);
	ROS_INFO("Z Error = %f\n", fabs(error.point.z));
	ROS_INFO("***********************************************************************\n");

	sendCommand(error);
	mErrorPub.publish(error);
}

void VisServo::sendCommand(const PointStamped error)
{
	float threshold = 0.01;
	if (fabs(error.point.z) > threshold)
	{
		ROS_INFO("***********************************************************************\n");
		ROS_INFO("Z Error = %f\n", fabs(error.point.z));
		ROS_INFO("***********************************************************************\n");
		
		// sensor_msgs::JointState command;
		// command.header = std_msgs::Header();
		// command.position[0] = 0.0;
		// command.position[1] = 2.0;
		// command.position[2] = 0.0;
		// mCommandPub.publish(command);

		mCmd[0].actuatorCommand().setPosition(0.0);
		mCmd[1].actuatorCommand().setPosition(2.0);
		mCmd[2].actuatorCommand().setPosition(0.0);
		mGroup_g->sendCommand(mCmd);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kdc_servo");

	std::string cameraTopic;
	std::string boardTopic;
	
	ros::NodeHandle nh("~");
	nh.getParam("cameraTopic", cameraTopic);
	nh.getParam("boardTopic", boardTopic);

	VisServo* pVisServo = new VisServo();

	int buffer = 1;
	message_filters::Subscriber<PointStamped> cameraPointSub(nh, cameraTopic, buffer);
	message_filters::Subscriber<PointStamped> boardPointSub(nh, boardTopic, buffer);

	typedef sync_policies::ApproximateTime<PointStamped, PointStamped> VisualServo;
	Synchronizer<VisualServo> sync(VisualServo(10), cameraPointSub, boardPointSub);
	
	// typedef void (VisServo::*OverloadError)(const PointStamped&, const PointStamped&);
	sync.registerCallback(boost::bind(&VisServo::calcError, pVisServo, _1, _2));

	ros::spin();
	return 0;
}