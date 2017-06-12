#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>
#include <string>
#include <vector>
#include <math.h>

const float TIRE_RADIUS = 0.18415;
const float TRACK = 0.43457;
const float YAW_RATE_MULTIPLIER = 0.255;  //This is a workaround until we can characterize our yaw rate wrt to our delta Vs

std::vector<double> currentPosition (4, 0);
std::vector<double> currentVelocity (4, 0);
std::vector<double> previousPosition (4, 0);
std::vector<double> previousVelocity (4, 0);

ros::Time current_time;

// double x;
// double y;
// double th;

// double vx;

/**
 *  Calculate odometry here!
 **/
void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  /* - Example of joint_states
    header: 
    seq: 16875
    stamp: 
      secs: 1465574934
      nsecs: 330986984
    frame_id: ''
  name: ['Front_Left_Drive', 'Rear_Left_Drive', 'Front_Right_Drive', 'Rear_Right_Drive']
  position: [-9.12737471262087, -9.131563250218527, 12.9224186022971, 12.918229111025127]
  velocity: [0.0, 0.0, -0.0, -0.0]
  effort: [0.0, 0.0, 0.0, 0.0]
  */

  // Grab current position and velocity, fl rl fr rr
  // ROS_INFO_STREAM("Got into callback!");
  for (int i = 0; i < 4; i++)
  {
    currentPosition[i] = (msg->position[i] * TIRE_RADIUS);  //convert to linear position
    currentVelocity[i] = (msg->velocity[i] * TIRE_RADIUS);  //convert to linear velocity
    // std::cout << "currentPosition:   " << currentPosition[i] << ";   current velocity:  " << currentVelocity[i] << std::endl;
  }

  // Get current time
  current_time = ros::Time::now();
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "boost_odom_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("wheel_encoder/odom", 1);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time last_time;
  last_time = ros::Time::now();

  ros::Subscriber sub_joint_command = n.subscribe("joint_states", 1, jointstateCallback);

  // Sleep to allow time for joint_states to be updated, particularly position of joints, necessary?
  sleep(5);  
  ros::spinOnce();

  // for (int i = 0; i < 4; i++)
  //   {
  //   std::cout << lastPosition[i] << std::endl;
  //   }

  // Update last position with initial position from Hebi motor controllers
  std::cout << "Initial Position of Drive Motors is (FL, RL, FR, RR): ";
  for (int i = 0; i < 4; i++)
  {
    previousPosition[i] = currentPosition[i];
    std::cout << previousPosition[i] << "   ";
    // std::cout << "currentPosition:   " << currentPosition[i] << ";   current velocity:  " << currentVelocity[i] << std::endl;
  }
  std::cout << std::endl;
  // sleep(2);

  // Set initial positions to 0
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Rate r(50.0);  // Publishing rate, in Hz
  while (ros::ok())
  {
    // Compute odometry in a typical way given the velocities of the robot
    //  take the average velocity over the interval, as the current velocity may not be an accurate representation
    //  of the velocity between updates 
    double dt = (current_time - last_time).toSec();

    // ROS_INFO("dt is %5.10f!", dt);
    // check for division by zero
    if (dt < 1e-6) {
      ROS_INFO("Time step in boost_odom is nearly zero, dt is %5.10f!", dt);
      last_time = current_time;
      ros::spinOnce();
      continue;
    }

    // Compute average velocity, fl rl fr rr
    double leftAvgVel = ((currentPosition[0] - previousPosition[0]) / dt + \
                        (currentPosition[1] - previousPosition[1]) / dt) / 2;
    double rightAvgVel = ((currentPosition[2] - previousPosition[2]) / dt + \
                        (currentPosition[3] - previousPosition[3]) / dt) / 2;

    // Compute average change in position
    double leftAvgDeltaPos = ((currentPosition[0] - previousPosition[0]) + \
                        (currentPosition[1] - previousPosition[1])) / 2;
    double rightAvgDeltaPos = ((currentPosition[2] - previousPosition[2]) + \
                        (currentPosition[3] - previousPosition[3])) / 2;

    // Compute odometry using wheel velocities calculated above, all in the robot frame
    double deltaX = (leftAvgDeltaPos + rightAvgDeltaPos) / 2;
    double deltaTh = (rightAvgDeltaPos - leftAvgDeltaPos) / TRACK * YAW_RATE_MULTIPLIER;

    // Store previous position for odom velocity calculation further down
    double prevX = x;
    double prevY = y;
    double prevTh = th;
    
    // Check if the vehicle has not moved or is moving in a straight line
    const double minX = 1e-6;
    const double minTh = 1e-6;
    // std::cout << "Delta Theta: " << deltaTh << "    Delta X: " << deltaX << std::endl;
    // std::cout << " (x, y, th) -> (" << x << ", " << y << ", " << th << ")" << std::endl;

    if ((fabs(deltaTh) > minTh) && (fabs(deltaX) > minX)) {  // Moving forward and turning at the same time

      x += TRACK / 2 * ( ( rightAvgVel + leftAvgVel ) / ( rightAvgVel - leftAvgVel ) ) * \
            ( sin( ( rightAvgDeltaPos - leftAvgDeltaPos ) / TRACK + th ) - sin( th ) );
      y += -TRACK / 2 * ( ( rightAvgVel + leftAvgVel ) / ( rightAvgVel - leftAvgVel ) ) * \
            ( cos ( ( rightAvgDeltaPos - leftAvgDeltaPos ) / TRACK + th ) - cos( th ) );
      th += deltaTh;

    } else if ((fabs(deltaTh) < minTh) && (fabs(deltaX) > minX)) {  // Moving in a straight line

      x += deltaX * cos(th);
      y += -deltaX * sin(th);

    } else if ((fabs(deltaTh) > minTh) && (fabs(deltaX) < minX)) {  // Zero-point turning

      th += deltaTh;

    } else if ((fabs(deltaTh) < minTh) && (fabs(deltaX) < minX)) {  // Not moving

      // Just here as a placeholder, remove later?

    } else {  // Error checking
      ROS_INFO("Robot is doing something weird, check odom calcs");
      continue;
    }

    /**
     *  NO NEED TO PUBLISH THIS TF, ROBOT_LOCALIZATION WILL DO THAT
    **/
    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "wheel_encoder/odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "wheel_encoder/odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity IN THE BODY FRAME FOR ROBOT_LOCALIZATION 
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = (leftAvgVel + rightAvgVel) / 2;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = (rightAvgVel - leftAvgVel) / TRACK * YAW_RATE_MULTIPLIER;;

    if (fabs(odom.twist.twist.linear.x) <= 1e-3 && fabs(odom.twist.twist.angular.z) <= 1e-3) //robot is stationary
    {
      //set the covariance IN THE BODY FRAME
      /**
       * The values are ordered as x, y, z, roll, pitch, yaw.
       * 
      **/
      odom.twist.covariance[0] = 1e-9;
      odom.twist.covariance[7] = 1e-9;
      odom.twist.covariance[14] = 1e-9;
      odom.twist.covariance[21] = 1e6;
      odom.twist.covariance[28] = 1e6;
      odom.twist.covariance[35] = 1e-9;

      odom.pose.covariance[0] = 1e-9; 
      odom.pose.covariance[7] = 1e-9;
      odom.pose.covariance[14] = 1e-9;
      odom.pose.covariance[21] = 1e6;
      odom.pose.covariance[28] = 1e6;
      odom.pose.covariance[35] = 1e-9;
    }
    else // robot wheels are moving
    {
      odom.twist.covariance[0] = 1e-3;
      odom.twist.covariance[7] = 1e-3;
      odom.twist.covariance[14] = 1e-9;
      odom.twist.covariance[21] = 1e6;
      odom.twist.covariance[28] = 1e6;
      odom.twist.covariance[35] = 0.174;

      odom.pose.covariance[0] = 1.0; 
      odom.pose.covariance[7] = 1.0;
      odom.pose.covariance[14] = 1e-9;
      odom.pose.covariance[21] = 1e6;
      odom.pose.covariance[28] = 1e6;
      odom.pose.covariance[35] = 0.174;
    }

    //publish the message
    odom_pub.publish(odom);

    // Update previous position with current and start it all over again
    for (int i = 0; i < 4; i++)
      {
        previousPosition[i] = currentPosition[i];
        previousVelocity[i] = currentVelocity[i];
      }

    // Same with the time
    last_time = current_time;
    
    // perform subscribed callback again and wait until it has to publish again
    r.sleep();
    ros::spinOnce();  
  }
  return 0;

}
