#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "hebi/ArmServo.h"
#include <sstream>
#include <math.h>
#include <log4cxx/logger.h>

#include "servoVars.hpp"

int idleFlag = 0;
int finalMoveFlag = 0;
int counter;

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
    //ROS_INFO_STREAM("I'm in the callback");

    sensor_msgs::JointState output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "";

    //Calculate position of joint 2
    //float hyp = 0.3;
    float joint2_x = hyp*cos(jointStatePtr->position[4]);
    float joint2_y = hyp*sin(jointStatePtr->position[4]);
    ROS_DEBUG_STREAM("Joint2_x: " << joint2_x << ", Joint2_y: " << joint2_y);

    //Calculate camera position (I could subscribe to the topic but this is easier)

    //float hyp_camera = .245;
    float camera_x = hyp_camera*cos(jointStatePtr->position[4] + jointStatePtr->position[6]) + joint2_x;
    float camera_y = hyp_camera*sin(jointStatePtr->position[4] + jointStatePtr->position[6]) + joint2_y;
    ROS_DEBUG_STREAM("Camera_x: " << camera_x << ", Camera_y: " << camera_y);

    //Convert line segments to vectors
    float link2Vector_x = camera_x - joint2_x;
    float link2Vector_y = camera_y - joint2_y;
    float destVector_x = x_estimate - joint2_x;
    float destVector_y = y_estimate - joint2_y;

    //calculate angle between vectors
    float theta = acos((link2Vector_x*destVector_x + link2Vector_y*destVector_y)/(sqrt(link2Vector_x*link2Vector_x + link2Vector_y*link2Vector_y)*sqrt(destVector_x*destVector_x + destVector_y*destVector_y)));

    float line1 = atan2((joint2_y-camera_y),(joint2_x-camera_x));  //line1Y1-line1Y2,line1X1-line1X2
    float line2 = atan2((joint2_y-y_estimate),(joint2_x-x_estimate));
    float line_diff = line1-line2;
    ROS_DEBUG_STREAM("Line diff: " << line_diff);

    if (counter<50)
    {
        float pointAtPlant;
        if (direction == "left")
        {
            ROS_DEBUG_STREAM("Desired Angle of joint2: " << (jointStatePtr->position[6] + theta));
            ROS_DEBUG_STREAM("Before theta is: " << ((link2Vector_x*destVector_x + link2Vector_y*destVector_y)/(sqrt(link2Vector_x*link2Vector_x + link2Vector_y*link2Vector_y)*sqrt(destVector_x*destVector_x + destVector_y*destVector_y))));
            ROS_DEBUG_STREAM("THETA IS: " << theta);
            pointAtPlant = jointStatePtr->position[6] - line_diff;
        }
        else
        {
            ROS_DEBUG_STREAM("Desired Angle of joint2: " << (jointStatePtr->position[6] + theta));
            pointAtPlant = jointStatePtr->position[6] - line_diff;
        }

        output.position.push_back(NAN);
        output.position.push_back(NAN);
        output.position.push_back(NAN);
        output.position.push_back(NAN);
        output.position.push_back(NAN);
        output.position.push_back(NAN);
        output.position.push_back(pointAtPlant);
        pub_.publish(output);
    }

    else
    {

        //float L1 = 0.299;
        //float L2 = 0.285;

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

        if (direction == "left")
        {

            mPError1 = theta1_L-jointStatePtr->position[4];
            //mIError1 += theta1_L-jointStatePtr->position[0];

            mPError2 = theta2_L-jointStatePtr->position[6];
            //mIError2 += theta2_L-jointStatePtr->position[2];
        }

        else if (direction == "right")
        {

            mPError1 = theta1_R-jointStatePtr->position[4];
            mPError2 = theta2_R-jointStatePtr->position[6];
        }

        else
            ROS_FATAL("COMMAND WAS NEITHER LEFT NOR RIGHT!!!");

        float gain= float(counter)/500;//0.2;
        ROS_DEBUG_STREAM("Gain is: " << gain);


        float command1 = jointStatePtr->position[4]+gain*mPError1;
        float command2 = jointStatePtr->position[6]+gain*mPError2;

        //ROS_INFO_STREAM("Command to motor 21: " << command1 << " Command to motor 23: " << command2);


        //std::cout << "commanding: " << x_estimate << ", " << y_estimate << std::endl;
        if ((finalMoveFlag == 1) && (idleFlag ==0))
        {
            //ROS_INFO("finalMoveFlag is 1");
            output.position.push_back(NAN);
            output.position.push_back(NAN);
            output.position.push_back(NAN);
            output.position.push_back(NAN);                                    
            output.position.push_back(jointStatePtr->position[4]+mPError1);
            output.position.push_back(NAN);
            output.position.push_back(jointStatePtr->position[6]+mPError2);
        }

        /*
        else if ((idleFlag == 0) && (counter<50))
        {
            output.position.push_back(command1);
            output.position.push_back(1.0);
            output.position.push_back(command2);

            //output.velocity.push_back(jointStatePtr->position[0]+mPError1);
            //output.velocity.push_back(1.0);
            //output.velocity.push_back(jointStatePtr->position[2]+mPError2);
        }
        */

        else if (idleFlag == 0)
        {
            output.position.push_back(NAN);
            output.position.push_back(NAN);   
            output.position.push_back(NAN);
            output.position.push_back(NAN);                        
            output.position.push_back(command1);
            output.position.push_back(NAN);
            //output.position.push_back(command2);


            //--------------------------------JUST A TEST----------------------------
            //Calculate position of joint 2
            joint2_x = hyp*cos(command1);
            joint2_y = hyp*sin(command1);
            ROS_DEBUG_STREAM("Joint2_x: " << joint2_x << ", Joint2_y: " << joint2_y);

            //Calculate camera position (I could subscribe to the topic but this is easier)
            camera_x = hyp_camera*cos(command1 + jointStatePtr->position[6]) + joint2_x;
            camera_y = hyp_camera*sin(command1 + jointStatePtr->position[6]) + joint2_y;
            ROS_DEBUG_STREAM("Camera_x: " << camera_x << ", Camera_y: " << camera_y);

            line1 = atan2((joint2_y-camera_y),(joint2_x-camera_x));
            line2 = atan2((joint2_y-y_estimate),(joint2_x-x_estimate));
            line_diff = line1-line2;
            ROS_DEBUG_STREAM("Line diff: " << line_diff);
            //--------------------------------------------------------------------------


            output.position.push_back(jointStatePtr->position[6] - line_diff);

            ROS_DEBUG("******************************GOING FOR IT!!*****************************************\n");
            ROS_DEBUG("x_board=%f, y_board=%f \n", x_estimate, y_estimate);
            ROS_DEBUG("r = %f, alpha = %f, beta = %f\n", r, alpha, beta);
            ROS_DEBUG("theta1_L = %f, theta1_R = %f, theta2_L = %f, theta2_R = %f\n", theta1_L, theta1_R, theta2_L, theta2_R);
            ROS_DEBUG("Joint Positions: (%f, %f, %f)\n", jointStatePtr->position[4], jointStatePtr->position[5], jointStatePtr->position[6]);
            ROS_DEBUG("P Error_1 = %f, P Error_2 = %f\n", mPError1, mPError2);
            ROS_DEBUG_STREAM("Command1: " << command1 << ", Command2: " << command2);
            ROS_DEBUG("***********************************************************************\n");
            ROS_DEBUG_STREAM("Back in main, global counter is: " << counter);
            ROS_DEBUG_STREAM("Final move flag is: " << finalMoveFlag);

            //output.velocity.push_back(jointStatePtr->position[0]+mPError1);
            //output.velocity.push_back(1.0);
            //output.velocity.push_back(jointStatePtr->position[2]+mPError2);
        }

        else
        {
            //ROS_INFO("Idling the motors");
            output.position.push_back(NAN);
            output.position.push_back(NAN);
            output.position.push_back(NAN);
            output.position.push_back(NAN);            
            output.position.push_back(NAN);
            output.position.push_back(NAN);
            output.position.push_back(NAN);
            output.velocity.push_back(NAN);
            output.velocity.push_back(NAN);
            output.velocity.push_back(NAN);
            output.velocity.push_back(NAN);            
            output.velocity.push_back(0);
            output.velocity.push_back(0);
            output.velocity.push_back(0);

        }

        pub_.publish(output);
        //ROS_INFO_STREAM("Just published!");
    }
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
    counter = 0;

    SubscribeAndPublish SAPObject;

    SAPObject.getDestination(req.arm_servo_req_x, req.arm_servo_req_y, req.arm_servo_req_jointAngle);

    ros::Rate loop_rate(100);

    while (counter <50)    //Servo to the stalk for 50 cycles
    {
        ros::spinOnce();
        loop_rate.sleep();
        counter++;
    }

    sleep(1);

    while (counter <400)    //Servo to the stalk for 400 cycles
    {
        ros::spinOnce();
        loop_rate.sleep();
        counter++;
    }

    finalMoveFlag = 1;  //I'm seeing a fair amount of error after completing servoing, so this is a cheap (very hacky) fix
    for(int n=0; n<100; n++)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


    idleFlag = 1;      //I need to idle the motors after completing the stalk servoing
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

  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

  ros::ServiceServer service = n.advertiseService("arm_servo", servo);

  ros::spin();

  return 0;
}

