#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "hebi/ArmServo.h"
#include <sstream>

int idleFlag = 0;
int finalMoveFlag = 0;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::JointState>("/joint_commands", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/joint_states", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const sensor_msgs::JointState::ConstPtr& jointStatePtr)
  {
    ROS_INFO_STREAM("I'm in the callback");

    float L1 = 0.3;
    float L2 = 0.31;

    float theta1_L, theta1_R, theta2_L, theta2_R;
    float mPError1, mPError2;

    float r = sqrt(pow(x_estimate, 2)+pow(y_estimate, 2));
    float alpha = acosf((powf(L1,2)+powf(L2,2)-powf(r,2))/(2.0*L1*L2));
    float beta = acosf((powf(L1,2)-powf(L2,2)+powf(r,2))/(2.0*L1*r));

    theta2_L = M_PI + alpha;
    theta2_R = M_PI - alpha;

    theta1_L = atan2f(y_estimate, x_estimate) + beta;
    theta1_R = atan2f(y_estimate, x_estimate) - beta;

    theta1_L = wrapAngle(theta1_L);
    theta1_R = wrapAngle(theta1_R);
    theta2_L = wrapAngle(theta2_L);
    theta2_R = wrapAngle(theta2_R);

    sensor_msgs::JointState output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "";

    if (direction == "left")
    {

        mPError1 = theta1_L-jointStatePtr->position[0];
        //mIError1 += theta1_L-jointStatePtr->position[0];

        mPError2 = theta2_L-jointStatePtr->position[2];
        //mIError2 += theta2_L-jointStatePtr->position[2];
    }

    else if (direction == "right")
    {

        mPError1 = theta1_R-jointStatePtr->position[0];
        mPError2 = theta2_R-jointStatePtr->position[2];
    }

    else
        ROS_FATAL("COMMAND WAS NEITHER LEFT NOR RIGHT!!!");


    ROS_INFO("******************************GOING FOR IT!!*****************************************\n");
    ROS_INFO("x_board=%f, y_board=%f \n", x_estimate, y_estimate);
    ROS_INFO("r = %f, alpha = %f, beta = %f\n", r, alpha, beta);
    ROS_INFO("theta1_L = %f, theta1_R = %f, theta2_L = %f, theta2_R = %f\n", theta1_L, theta1_R, theta2_L, theta2_R);
    ROS_INFO("Joint Positions: (%f, %f, %f)\n", jointStatePtr->position[0], jointStatePtr->position[1], jointStatePtr->position[2]);
    ROS_INFO("P Error_1 = %f, P Error_2 = %f\n", mPError1, mPError2);
    ROS_INFO("***********************************************************************\n");
    //ROS_INFO_STREAM("Back in main, global counter is: " << globalcounter);

    float gain=0.2;


    float command1 = jointStatePtr->position[0]+gain*mPError1;
    float command2 = jointStatePtr->position[2]+gain*mPError2;

    ROS_INFO_STREAM("Command to motor 21: " << command1 << " Command to motor 23: " << command2);


    //std::cout << "commanding: " << x_estimate << ", " << y_estimate << std::endl;
    if ((finalMoveFlag == 1) && (idleFlag ==0))
    {
        ROS_INFO("finalMoveFlag is 1");
        output.position.push_back(jointStatePtr->position[0]+mPError1);
        output.position.push_back(1.0);
        output.position.push_back(jointStatePtr->position[2]+mPError2);
    }

    else if (idleFlag == 0)
    {
        output.position.push_back(command1);
        output.position.push_back(1.0);
        output.position.push_back(command2);
    }

    else
    {
        ROS_INFO("Idling the motors");
        output.position.push_back(NAN);
        output.position.push_back(1.0);
        output.position.push_back(NAN);
        output.velocity.push_back(0);
        output.velocity.push_back(0);
        output.velocity.push_back(0);

    }
    //.... do something with the input and generate the output...
    pub_.publish(output);
    ROS_INFO_STREAM("Just published!");
  }

  void getDestination(float x_dest, float y_dest, std::string dir)
  {
      x_estimate = x_dest;
      y_estimate = y_dest;
      direction = dir;
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

private:
  //float motor21;
  //float motor23;
  float x_estimate;
  float y_estimate;
  std::string direction;
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish


bool servo(hebi::ArmServo::Request  &req,
         hebi::ArmServo::Response &res)
{
    int counter = 0;

    SubscribeAndPublish SAPObject;

    SAPObject.getDestination(req.arm_servo_req_x, req.arm_servo_req_y, req.arm_servo_req_jointAngle);

    ros::Rate loop_rate(10);

    while (counter <50)
    {
        ros::spinOnce();
        loop_rate.sleep();
        counter++;
    }

    finalMoveFlag = 1;
    for(int n=0; n<10; n++)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    idleFlag = 1;
    for(int i=0; i<10; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    idleFlag = 0;
    finalMoveFlag = 0;

    res.arm_servo_resp = req.arm_servo_req_jointAngle;

    return true;
}

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "cylinder_servo");

  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("arm_servo", servo);

  ros::spin();

  return 0;
}

/*
float x_global;
float y_global;
float z_global;

void myCallback(const sensor_msgs::JointState::ConstPtr& jointStatePtr)
{
    ros::NodeHandle mNode;
    ros::Publisher chatter_pub = mNode.advertise<sensor_msgs::JointState>("/joint_commands", 1);
    sensor_msgs::JointState msg;

    msg.position.push_back(x_global);
    msg.position.push_back(y_global);
    msg.position.push_back(z_global);

    std::cout << x_global << " " << y_global << " " << z_global << " " << std::endl;

    chatter_pub.publish(msg);
}


bool servo(hebi::ArmServo::Request  &req,
         hebi::ArmServo::Response &res)
{
    x_global = req.arm_servo_req_x;
    y_global = req.arm_servo_req_y;
    z_global = req.arm_servo_req_z;

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/joint_states", 1, myCallback);

    ros::Rate loop_rate(10);

    int count = 0;
    while (count<50)
    {
      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }

    res.arm_servo_resp = req.arm_servo_req_jointAngle;

    return true;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("arm_servo", servo);

  ros::spin();

  return 0;
}
*/
