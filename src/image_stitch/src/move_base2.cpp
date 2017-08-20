#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <math.h>

int globalFlag = 0;
float initialLocation;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish();
    void callback(const nav_msgs::Odometry::ConstPtr& robotLoc);

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
};//End of class SubscribeAndPublish

SubscribeAndPublish::SubscribeAndPublish()
{
    this->pub=nh.advertise<geometry_msgs::Twist>("/camera_stitch/cmd_vel", 1);
    this->sub=nh.subscribe<nav_msgs::Odometry>("/wheel_encoder/odom", 1,&SubscribeAndPublish::callback, this);
}

/*
bool servo(hebi::ArmServo::Request  &req,
         hebi::ArmServo::Response &res)
{

    SubscribeAndPublish SAPObject;

    SAPObject.getDestination(req.arm_servo_req_x, req.arm_servo_req_y, req.arm_servo_req_jointAngle);

    ros::Rate loop_rate(100);

    //Do stuff

    res.arm_servo_resp = req.arm_servo_req_jointAngle;

    return true;
}
*/

void SubscribeAndPublish::callback(const nav_msgs::Odometry::ConstPtr& robotLoc)//sensor_msgs::JointState::ConstPtr& jointStatePtr)
{

    //ROS_DEBUG_STREAM("Location: " << robotLoc->pose.pose.position.x);
    //ROS_INFO_STREAM("I'm in the callback");
    if (globalFlag == 0) {
        initialLocation = robotLoc->pose.pose.position.x;
        globalFlag = 1;
    }

    float currentLocation = robotLoc->pose.pose.position.x;
    //float Front_Left_Drive = jointStatePtr->position[0];
    //float Front_Right_Drive = jointStatePtr->position[1];
    geometry_msgs::Twist output;

    if((currentLocation-initialLocation)<0.5){
        output.linear.x = 0.33;
        output.linear.y = 0.0;
        output.linear.z = 0.0;
        output.angular.x = 0.0;
        output.angular.y = 0.0;
        output.angular.z = 0.0;
    }
    else {
        output.linear.x = 0.0;
        output.linear.y = 0.0;
        output.linear.z = 0.0;
        output.angular.x = 0.0;
        output.angular.y = 0.0;
        output.angular.z = 0.0;
    }


    //ROS_DEBUG_STREAM("Front_Left_Drive: " << Front_Left_Drive << ", Front_Right_Drive: " << Front_Right_Drive);
    //std::cout << "Front_Left_Drive: " << Front_Left_Drive << ", Front_Right_Drive: " << Front_Right_Drive << std::endl;

    pub.publish(output);
    //ROS_INFO_STREAM("Just published!");

}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "move_base");

    ros::NodeHandle nh;
    nh.param<std::string>("port", port, "/dev/ttyACM1");
    nh.param("baud", baud, 115200);

    //ros::ServiceServer service = n.advertiseService("move_base", servo);

    SubscribeAndPublish SAPObject;


    syncboard.setTimeout(serial::Timeout::max(), 500, 0, 500, 0);
    syncboard.setPort(port);
    syncboard.setBaudrate(baud);
    syncboard.open();
    handshake();
    while(ros::ok()){
        if (continuous_started and syncboard.available() > NMEA_LENGTH) {
            std::string nmea_string;
            if(read_nmea_string(&nmea_string)){
                publish_nmea_string(nmea_string);
            }
        }
        ros::spinOnce();
        ros::Rate(100).sleep();
    }
    syncboard.write("s");

    //ros::spin();

    return 0;
}