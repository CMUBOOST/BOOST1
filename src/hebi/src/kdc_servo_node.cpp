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

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

hebi::Group* group_g = NULL;

class VisServo
{
public:
	VisServo();
	void calcError(const boost::shared_ptr<const PointStamped>& cameraPointPtr, const boost::shared_ptr<const PointStamped>& boardPointPtr, const boost::shared_ptr<const JointState>& jointStatePtr);
	void sendCommand(const PointStamped error, const PointStamped board, const PointStamped camera, const JointState joint);

private:
	ros::NodeHandle mNode;
	ros::Publisher mErrorPub;
	ros::Publisher mCommandPub;

	ros::Publisher mArmData;

	// static float mIError1, mIError2;
	float mPError1, mPError2, mIError1, mIError2;
};

VisServo::VisServo()
{
	int buffer = 1;
	mErrorPub = mNode.advertise<PointStamped>("kdc/error_topic", buffer);
	mCommandPub = mNode.advertise<sensor_msgs::JointState>("joint_commands", buffer);
	mArmData = mNode.advertise<std_msgs::Float32MultiArray>("kdc/arm_data", buffer);

	mPError1 = 0.0;
	mPError2 = 0.0;
	mIError1 = 0.0;
	mIError2 = 0.0;
}
// float VisServo::mIError1 = 0.0;
// float VisServo::mIError2 = 0.0;

void VisServo::calcError(const boost::shared_ptr<const PointStamped>& cameraPointPtr, const boost::shared_ptr<const PointStamped>& boardPointPtr, const boost::shared_ptr<const JointState>& jointStatePtr)
{
	PointStamped camera = *cameraPointPtr;
	PointStamped board = *boardPointPtr;
	JointState joints = *jointStatePtr;

	PointStamped error;
	error.header = std_msgs::Header();
	error.point.x = camera.point.x - board.point.x;
	error.point.y = camera.point.y - board.point.y;
	error.point.z = camera.point.z - board.point.z;
	
	// ROS_INFO("***********************************************************************\n");
	// ROS_INFO("Camera Point: (%f, %f, %f)\n", camera.point.x, camera.point.y, camera.point.z);
	// ROS_INFO("Board Point: (%f, %f, %f)\n", board.point.x, board.point.y, board.point.z);
	// ROS_INFO("Error Point: (%f, %f, %f)\n", error.point.x, error.point.y, error.point.z);
	// ROS_INFO("Z Error = %f\n", fabs(error.point.z));
	// ROS_INFO("***********************************************************************\n");

	sendCommand(error, board, camera, joints);
	mErrorPub.publish(error);
}

float wrapAngle(float angle)
{
	if (angle > M_PI)
	{
		angle -= 2*M_PI;
	}

	if (angle < -M_PI)
	{
		angle += M_PI;
	}

	return angle; 
}

void VisServo::sendCommand(const PointStamped error, const PointStamped board, const PointStamped camera, const JointState joint)
{
	float threshold = 0.01;
	float xythreshold = 0.001;

	float L1 = 0.277;
    float L2 = 0.45;

	float KP = 5.0;
	float KI = 2.0;
	// float KP = 15.0;
	// float KI = 12.0;
	float dt = ros::Time::now().toSec() - joint.header.stamp.toSec();

	sensor_msgs::JointState msg;
	msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";

    float zVelocity = 0.0;
    float theta1_L, theta1_R, theta2_L, theta2_R;

	//theta2_L = -120.0*(M_PI/180.0);
	//theta2_R = -120.0*(M_PI/180.0);

	//theta1_L = M_PI_2;
	//theta1_R = M_PI_2;

	if (sqrt(pow(error.point.x,2)+pow(error.point.y,2)) > xythreshold)
	{
		float r = sqrt(pow(board.point.x, 2)+pow(board.point.y, 2));
		float alpha = acosf((powf(L1,2)+powf(L2,2)-powf(r,2))/(2.0*L1*L2));
		float beta = acosf((powf(L1,2)-powf(L2,2)+powf(r,2))/(2.0*L1*r));

		theta2_L = M_PI + alpha;
		theta2_R = M_PI - alpha;

		theta1_L = atan2f(board.point.y, board.point.x) + beta;
		theta1_R = atan2f(board.point.y, board.point.x) - beta;

		theta1_L = wrapAngle(theta1_L);
		theta1_R = wrapAngle(theta1_R);
		theta2_L = wrapAngle(theta2_L);
		theta2_R = wrapAngle(theta2_R);

		mPError1 = theta1_L-joint.position[0];
		mIError1 += theta1_L-joint.position[0];

		mPError2 = theta2_L-joint.position[2];
		mIError2 += theta2_L-joint.position[2];

		ROS_INFO("***********************************************************************\n");
		ROS_INFO("x_board=%f, y_board=%f \n", board.point.x, board.point.y);
		ROS_INFO("x_camera=%f, y_camera=%f \n", camera.point.x, camera.point.y);
		ROS_INFO("r = %f, alpha = %f, beta = %f\n", r, alpha, beta);
		ROS_INFO("theta1_L = %f, theta1_R = %f, theta2_L = %f, theta2_R = %f\n", theta1_L, theta1_R, theta2_L, theta2_R);
		ROS_INFO("Joint Positions: (%f, %f, %f)\n", joint.position[0], joint.position[1], joint.position[2]);
		ROS_INFO("TIME STEP = %f\n", dt);
		ROS_INFO("P Error, 1 = %f, I Error, 1 = %f, P Error, 2 = %f, I Error, 2 = %f\n", mPError1, mIError1, mPError2, mIError2);
		ROS_INFO("***********************************************************************\n");
	}
	//else
	//{
	//	theta2_L = 10.0*(M_PI/180.0);
	//	theta2_R = 10.0*(M_PI/180.0);

	//	theta1_L = M_PI_2;
	//	theta1_R = M_PI_2;

	//	ROS_INFO("***********************************************************************\n");
	//	ROS_INFO("theta1_L = %f, theta1_R = %f, theta2_L = %f, theta2_R = %f\n", theta1_L, theta1_R, theta2_L, theta2_R);
	//	ROS_INFO("***********************************************************************\n");
	//}

	
	if (fabs(error.point.z) > threshold)
	{
		// ROS_INFO("***********************************************************************\n");
		// ROS_INFO("Z Error = %f\n", fabs(error.point.z));
		// ROS_INFO("***********************************************************************\n");
		float KP = -10;
		zVelocity = KP*error.point.z;

		// msg.velocity.push_back(0.0);
  //   	msg.velocity.push_back(KP*error.point.z);
  //   	msg.velocity.push_back(0.0);
        // for (int i = 0; i < joint_names.size(); i++)
        // {
        //   msg.name.push_back(joint_names[i].c_str());
        //   if ((*fbk)[i].actuatorFeedback().hasPosition())
        //     // Dirty, fix this later, deals with linear stage positioning
        //     if ((i == 1)) 
        //       msg.position.push_back((*fbk)[i].actuatorFeedback().getPosition()/STAGE_GEAR_RATIO);
        //     else 
        //       msg.position.push_back((*fbk)[i].actuatorFeedback().getPosition());
        //   if ((*fbk)[i].actuatorFeedback().hasVelocity())
        //     // Dirty, fix this later, deals with linear stage positioning
        //     if ((i == 1))
        //       msg.velocity.push_back((*fbk)[i].actuatorFeedback().getVelocity()/STAGE_GEAR_RATIO);
        //     else
        //       msg.velocity.push_back((*fbk)[i].actuatorFeedback().getVelocity());
        //   if ((*fbk)[i].actuatorFeedback().hasTorque())
        //     // Dirty, fix this later, deals with linear stage positioning
        //     if ((i == 1))
        //       msg.effort.push_back((*fbk)[i].actuatorFeedback().getTorque()*STAGE_GEAR_RATIO);
        //     else
        //       msg.effort.push_back((*fbk)[i].actuatorFeedback().getTorque());
        // }
	}
	// else
	// {
	// 	msg.velocity.push_back(0.0);
	// 	msg.velocity.push_back(0.0);
	// 	msg.velocity.push_back(0.0);
	// 	mCommandPub.publish(msg);

	// 	msg.position.push_back(nanf(""));
	// 	msg.position.push_back(nanf(""));
	// 	msg.position.push_back(nanf(""));
	// }
	
	 //msg.position.push_back(theta1_L);
	 //msg.position.push_back(nanf(""));
	 //msg.position.push_back(theta2_L);

	float command1 = KP*mPError1 + KI*mIError1*dt;
	float command2 = KP*mPError2 + KI*mIError2*dt;

	msg.velocity.push_back(command1);
	msg.velocity.push_back(zVelocity);
	msg.velocity.push_back(command2);

	mCommandPub.publish(msg);


    std_msgs::Float32MultiArray armData;
    armData.data.clear();
    armData.data.push_back(theta1_L);
    armData.data.push_back(theta2_R);
    armData.data.push_back(theta1_R);
    armData.data.push_back(theta2_L);
    armData.data.push_back(board.point.x);
    armData.data.push_back(board.point.y);
    armData.data.push_back(joint.position[0]);
    armData.data.push_back(joint.position[2]);
    mArmData.publish(armData);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kdc_servo");

	std::string cameraTopic;
	std::string boardTopic;
	std::string jointTopic;
	
	ros::NodeHandle nh("~");
	nh.getParam("cameraTopic", cameraTopic);
	nh.getParam("boardTopic", boardTopic);
	nh.getParam("jointTopic", jointTopic);

	VisServo* pVisServo = new VisServo();

	int buffer = 1;
	message_filters::Subscriber<PointStamped> cameraPointSub(nh, cameraTopic, buffer);
	message_filters::Subscriber<PointStamped> boardPointSub(nh, boardTopic, buffer);
	message_filters::Subscriber<JointState> joint_statesSub(nh, jointTopic, buffer);

	typedef sync_policies::ApproximateTime<PointStamped, PointStamped, JointState> VisualServo;
	Synchronizer<VisualServo> sync(VisualServo(3), cameraPointSub, boardPointSub, joint_statesSub);
	
	// typedef void (VisServo::*OverloadError)(const PointStamped&, const PointStamped&);
	sync.registerCallback(boost::bind(&VisServo::calcError, pVisServo, _1, _2, _3));

	ros::spin();
	return 0;
}
