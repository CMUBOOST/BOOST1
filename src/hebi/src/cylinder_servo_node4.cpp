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
#include <unistd.h>
#include <message_filters/time_synchronizer.h>
#include "hebi/ArmServo.h"


#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

using namespace message_filters;
using namespace geometry_msgs;
using namespace sensor_msgs;

//hebi::Group* group_g = NULL;

int globalcounter = 1;
int total_steps = 40;
//int recognition_steps = 15;
float estimate_x;
float estimate_y;
std::string direction;

//float x_loc;
//float y_loc;
//float z_loc;

/*
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

void VisServo::calcError(const boost::shared_ptr<const PointStamped>& cameraPointPtr, const boost::shared_ptr<const PointStamped>& boardPointPtr, const boost::shared_ptr<const JointState>& jointStatePtr)
{
    if (globalcounter < recognition_steps)
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
}


void VisServo::sendCommand(const PointStamped error, const PointStamped board, const PointStamped camera, const JointState joint)
{
    float threshold = 0.01;
    float xythreshold = 0.001;

    float L1 = 0.3; //0.277;
    float L2 = 0.48; //0.45;

    float KP = 5.0;
    float KI = 2.0;

    float dt = ros::Time::now().toSec() - joint.header.stamp.toSec();
    std::cerr << "dt is " << ros::Time::now().toSec() - camera.header.stamp.toSec() << std::endl;

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";

    float zVelocity = 0.0;
    float theta1_L, theta1_R, theta2_L, theta2_R;


    float r = sqrt(pow(board.point.x, 2)+pow(board.point.y, 2));
    float alpha = acosf((powf(L1,2)+powf(L2,2)-powf(r,2))/(2.0*L1*L2));
    float beta = acosf((powf(L1,2)-powf(L2,2)+powf(r,2))/(2.0*L1*r));

    estimate_x = board.point.x;
    estimate_y = board.point.y;

    theta2_L = M_PI + alpha;
    theta2_R = M_PI - alpha;

    theta1_L = atan2f(board.point.y, board.point.x) + beta;
    theta1_R = atan2f(board.point.y, board.point.x) - beta;

    theta1_L = wrapAngle(theta1_L);
    theta1_R = wrapAngle(theta1_R);
    theta2_L = wrapAngle(theta2_L);
    theta2_R = wrapAngle(theta2_R);

    if (direction == "left")
    {

        mPError1 = theta1_L-joint.position[0];
        mIError1 += theta1_L-joint.position[0];

        mPError2 = theta2_L-joint.position[2];
        mIError2 += theta2_L-joint.position[2];
    }

    else if (direction == "right")
    {

        mPError1 = theta1_R-joint.position[0];
        mIError1 += theta1_R-joint.position[0];

        mPError2 = theta2_R-joint.position[2];
        mIError2 += theta2_R-joint.position[2];

    }

    else
        ROS_FATAL("COMMAND WAS NEITHER LEFT NOR RIGHT!!!");

    ROS_INFO("***********************************************************************\n");
    ROS_INFO("x_board=%f, y_board=%f \n", board.point.x, board.point.y);
    ROS_INFO("x_camera=%f, y_camera=%f \n", camera.point.x, camera.point.y);
    ROS_INFO("r = %f, alpha = %f, beta = %f\n", r, alpha, beta);
    ROS_INFO("theta1_L = %f, theta1_R = %f, theta2_L = %f, theta2_R = %f\n", theta1_L, theta1_R, theta2_L, theta2_R);
    ROS_INFO("Joint Positions: (%f, %f, %f)\n", joint.position[0], joint.position[1], joint.position[2]);
    ROS_INFO("TIME STEP = %f\n", dt);
    ROS_INFO("P Error, 1 = %f, I Error, 1 = %f, P Error, 2 = %f, I Error, 2 = %f\n", mPError1, mIError1, mPError2, mIError2);
    ROS_INFO("***********************************************************************\n");

    float gain=0.2;

    float command1 = joint.position[0]+gain*mPError1;
    float command2 = joint.position[2]+gain*mPError2;

    ROS_INFO_STREAM("Command to motor 21: " << command1 << " Command to motor 23: " << command2);

    //msg.position.push_back(0.5); //this is motor 21, and the units are in radians
    msg.position.push_back(command1);
    msg.position.push_back(1);
    //msg.position.push_back(-2.1);
    msg.position.push_back(command2);
    mCommandPub.publish(msg);

    globalcounter++;
    ROS_INFO_STREAM("global counter is: " << globalcounter);
    usleep(200000);

}
*/

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


void myCallback(const boost::shared_ptr<const JointState>& jointStatePtr)
{
    //ROS_INFO_STREAM("I heard success" << jointStatePtr->position[0]);
    ROS_INFO_STREAM("I'm in the callback");

    int buffer = 1;
    ros::Publisher mCommandPub;
    ros::NodeHandle mNode;
    mCommandPub = mNode.advertise<sensor_msgs::JointState>("joint_commands", buffer);
    ROS_INFO_STREAM("I'm in the second while-loop!");
    float L1 = 0.3;
    float L2 = 0.31; //.35;

    float theta1_L, theta1_R, theta2_L, theta2_R;
    float mPError1, mPError2, mIError1, mIError2;

    float r = sqrt(pow(estimate_x, 2)+pow(estimate_y, 2));
    float alpha = acosf((powf(L1,2)+powf(L2,2)-powf(r,2))/(2.0*L1*L2));
    float beta = acosf((powf(L1,2)-powf(L2,2)+powf(r,2))/(2.0*L1*r));

    if (isnan(alpha) || isnan(beta))
        ROS_ERROR("Stalk is too far away!!");

    theta2_L = M_PI + alpha;
    theta2_R = M_PI - alpha;

    theta1_L = atan2f(estimate_y, estimate_x) + beta;
    theta1_R = atan2f(estimate_y, estimate_x) - beta;

    theta1_L = wrapAngle(theta1_L);
    theta1_R = wrapAngle(theta1_R);
    theta2_L = wrapAngle(theta2_L);
    theta2_R = wrapAngle(theta2_R);

    ROS_INFO_STREAM("theta1_L :" << theta1_L << "theta1_R :" << theta1_R << "theta2_L :" << theta2_L << "theta2_R :" << theta2_R);

    const JointState joint;

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";

    //mPError1 = theta1_L-jointStatePtr->position[0];
    //mPError2 = theta2_L-jointStatePtr->position[2];


    if (direction == "left")
    {

        mPError1 = theta1_L-jointStatePtr->position[0];
        mIError1 += theta1_L-jointStatePtr->position[0];

        mPError2 = theta2_L-jointStatePtr->position[2];
        mIError2 += theta2_L-jointStatePtr->position[2];
    }

    else if (direction == "right")
    {

        mPError1 = theta1_R-jointStatePtr->position[0];
        mPError2 = theta2_R-jointStatePtr->position[2];
    }

    else
        ROS_FATAL("COMMAND WAS NEITHER LEFT NOR RIGHT!!!");


    ROS_INFO("******************************GOING FOR IT!!*****************************************\n");
    ROS_INFO("x_board=%f, y_board=%f \n", estimate_x, estimate_y);
    ROS_INFO("r = %f, alpha = %f, beta = %f\n", r, alpha, beta);
    ROS_INFO("theta1_L = %f, theta1_R = %f, theta2_L = %f, theta2_R = %f\n", theta1_L, theta1_R, theta2_L, theta2_R);
    ROS_INFO("Joint Positions: (%f, %f, %f)\n", jointStatePtr->position[0], jointStatePtr->position[1], jointStatePtr->position[2]);
    ROS_INFO("P Error_1 = %f, P Error_2 = %f\n", mPError1, mPError2);
    ROS_INFO("***********************************************************************\n");
    ROS_INFO_STREAM("Back in main, global counter is: " << globalcounter);

    float gain=0.2;

    float command1 = jointStatePtr->position[0]+gain*mPError1;
    float command2 = jointStatePtr->position[2]+gain*mPError2;

    ROS_INFO_STREAM("Command to motor 21: " << command1 << " Command to motor 23: " << command2);

    //ROS_INFO_STREAM("Command to motor 21: " << command1 << " Command to motor 23: " << command2);

    //msg.position.push_back(1.5); //this is motor 21, and the units are in radians
    //msg.position.push_back(command1);
    //msg.position.push_back(1);
    //msg.position.push_back(-1.8);

    msg.position = {command1, 1, command2};

    //msg.position.push_back(command2);
    //ros::Publisher mCommandPub;
    //mCommandPub = mNode.advertise<sensor_msgs::JointState>("joint_commands", buffer);
    mCommandPub.publish(msg);
    ROS_INFO_STREAM("Just published!");

    globalcounter++;
    usleep(100000);

    if (globalcounter == total_steps)
    {
        sensor_msgs::JointState msg2;
        msg2.header.stamp = ros::Time::now();
        msg2.header.frame_id = "";
        msg2.position.push_back(NAN);
        msg2.position.push_back(1);
        msg2.position.push_back(NAN);
        mCommandPub.publish(msg2);
    }

}


bool servo(hebi::ArmServo::Request  &req,
         hebi::ArmServo::Response &res)
{

  direction = req.arm_servo_req_jointAngle;
  /*
  res.arm_servo_resp = req.arm_servo_req;
  ROS_INFO_STREAM("request: " << req.arm_servo_req);

  ros::NodeHandle nh("~");

  VisServo* pVisServo = new VisServo();

  int buffer = 1;
  message_filters::Subscriber<PointStamped> cameraPointSub(nh, "/kdc/camera_point", buffer);
  message_filters::Subscriber<PointStamped> boardPointSub(nh, "/kdc/board_point_alpha", buffer);
  message_filters::Subscriber<JointState> joint_statesSub(nh, "/joint_states", buffer);

  typedef sync_policies::ApproximateTime<PointStamped, PointStamped, JointState> VisualServo;
  Synchronizer<VisualServo> sync(VisualServo(3), cameraPointSub, boardPointSub, joint_statesSub);

  sync.registerCallback(boost::bind(&VisServo::calcError, pVisServo, _1, _2, _3));

  while (globalcounter < recognition_steps)
  {
    ros::spinOnce();
  }

  ROS_INFO_STREAM("Out of the main while loop");
  */

  estimate_x = req.arm_servo_req_x;
  estimate_y = req.arm_servo_req_y;

  ROS_INFO_STREAM("estimate_x: " << estimate_x);
  ROS_INFO_STREAM("estimate_y: " << estimate_y);
  //z_loc = req.arm_servo_req_z;

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/joint_states", 1, myCallback); //now, instead of subscribing to board_point and camera_point, I just subscribe to joint_states

  ros::Rate loop_rate(5);

  while (globalcounter < total_steps)
  {
    ros::spinOnce();
    ROS_INFO_STREAM("globalcounter is: " << globalcounter);
    loop_rate.sleep();
  }

  globalcounter = 1;

  ROS_INFO_STREAM("Sending back servo success response: " << res.arm_servo_resp);
  return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "kdc_servo");

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("arm_servo", servo);
    ROS_INFO("Ready to SERVO!!");
    ros::spin();

    return 0;
}
