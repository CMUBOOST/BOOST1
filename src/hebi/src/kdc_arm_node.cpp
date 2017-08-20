#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include <geometry_msgs/Twist.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

#include <sstream>
#include <string>
#include <vector>
#include <math.h>

//const int MAX_RAW_VELOCITY = 50;
const double MAX_RAW_VELOCITY = 1.5;

const double MAX_POSITION_23 = 2.45;
const double MIN_POSITION_23 = -3.14;  // Code below assumes this is negative

const double MAX_POSITION_21 = 1.2;
const double MIN_POSITION_21 = -1.2;  // Code below assumes this is negative

const double MAX_POSITION_22 = 30;
const double MIN_POSITION_22 = -7.5; 

const double STAGE_GEAR_RATIO = 48.33;  // [rad/m] - 130mm per revolution (2pi)

/**
 * A short function to get name and family from a name split with a "|".
 */
bool split(const std::string &orig, std::string &name, std::string &family)
{
  std::stringstream ss(orig);
  if (!std::getline(ss, name, '|'))
    return false;
  std::getline(ss, family);
  return true;
}

// Global 'group' pointer so we can access this in a callback...ugly, but until
// we class-ify the node this will work.
hebi::Group* group_g = NULL;

/**
 * Controls the position of the first and second link, and applies velocity control to the stage motor
 */
int sendJointCommands(double position23, double velocity22, double position21)
{
  std::cout << "Joint Command - position23:" << position23 << " velocity22:" << velocity22 << " position21:" << position21 << "\n";

  // Sanity check velocity.
  if (((position23 > MAX_POSITION_23) || (position23 < MIN_POSITION_23))) {
    ROS_INFO("Position (23) out of range\n");
    return(-1);
  }
  // Sanity check velocity.
  if (fabs(velocity22) > MAX_RAW_VELOCITY) {
    ROS_INFO("Velocity (22) out of range\n");
    return(-1);
  }
  // Sanity check velocity.
  if (((position21 > MAX_POSITION_21) || (position21 < MIN_POSITION_21))) {
    ROS_INFO("Position (21) out of range\n");
    return(-1);
  }

  double motor22, motor23, motor21;

  motor22 = velocity22;

  if (((velocity22 == 0) || (velocity22 == -0))) {
    motor22 = 0;
  } 
  //std::cerr << "Actuating!!";
  motor22 = velocity22;
  motor23 =  position23;
  motor21 = position21;


  hebi::GroupCommand cmd(3);
  cmd[0].actuatorCommand().setPosition(motor21);
  cmd[1].actuatorCommand().setVelocity(motor22);
  cmd[2].actuatorCommand().setPosition(motor23);
  group_g->sendCommand(cmd);
  std::cerr << "Joint Commands:\t" << motor22 <<
            "\t" << motor23 << "\t" << motor21 << ".\n";

  return(1);
}

/**
 * Callback for joystick commands
 */

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  double velocity22, position23, position21;

  // Analog inputs
  const int control23 = 1;
  const int control21 = 4;

  // Discrete inputs
  const int TOP_RIGHT_BUTTON = 5;
  const int TOP_LEFT_BUTTON = 4;

  const int STAGE_UP = 13;
  const int STAGE_DOWN = 14;

  const double DEADBAND = 0.05; //0.4;

  // DEADMAN SWITCH IS TOP LEFT BUTTON
  if (msg->buttons[TOP_LEFT_BUTTON] > 0) {

  	// Velocity control of stage motor
  	if (msg->buttons[STAGE_UP] > 0){
    	velocity22 = MAX_RAW_VELOCITY;
    }

    if (msg->buttons[STAGE_DOWN] > 0){
    	velocity22 = -MAX_RAW_VELOCITY;
    }

    // Position control of 1st link
    position23 = msg->axes[control23];
    if (position23 > 0) {
    	position23 = position23 * MAX_POSITION_23;
    }  
    else if (position23 < 0) {
    	position23 = -position23 * MIN_POSITION_23;
    }
    if (abs(position23) < DEADBAND)  {
    	position23 = 0;
     }

    // Position control of 2nd link
    position21 = msg->axes[control21];
    if (position21 > 0) {
    	position21 = position21 * MAX_POSITION_21;
    }  
    else if (position23 < 0) {
    	position21 = -position21 * MIN_POSITION_21;
    }
    if (abs(position23) < DEADBAND)  {
    	position23 = 0;
     }

  } else {
    std::cout << "stop mode\n";
    // velocity22 = 0;
    // position23 = 0;
    // position21 = 0;
    velocity22 = nan("");
    position23 = nan("");
    position21 = nan("");
  }

  sendJointCommands(position23, velocity22, position21);
}


/**
 * callback for joint commands (ie low level command for each axis)
 */
void commandCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // TODO: check names instead of just length?
  if ((group_g->size() != msg->position.size() && msg->position.size() != 0) ||
      (group_g->size() != msg->velocity.size() && msg->velocity.size() != 0) ||
      (group_g->size() != msg->effort.size() && msg->effort.size() != 0))
  {
    ROS_INFO("Command length did not match group size.");
    std::cerr << "Command length did not match group size.";
    return;
  }
  hebi::GroupCommand cmd(group_g->size());
  for (int i = 0; i < msg->position.size(); i++){
    std::cout << "Position command:" << msg->position[i] << "\n";
    cmd[i].actuatorCommand().setPosition(msg->position[i]);
  }
  for (int i = 0; i < msg->velocity.size(); i++){
    std::cout << "Velocity command:" << msg->velocity[i] << "\n";
    cmd[i].actuatorCommand().setVelocity(msg->velocity[i]);
  }
  for (int i = 0; i < msg->effort.size(); i++)
    cmd[i].actuatorCommand().setTorque(msg->effort[i]);
  group_g->sendCommand(cmd);
}

/**
 * This node publishes feedback from named joints in the URDF model on the
 * parameter server under "robot_description".
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "kdc_arm");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Get the names of non-fixed joints in the model:
  std::vector<std::string> joint_names = {"X-00021", "X-00022", "X-00023"};
  std::vector<std::string> family_names = {"BOOST", "BOOST", "BOOST"};
  // This is for gazebo to recognize the joints
  // std::vector<std::string> joint_names_ns = {"boost_stalker::X-00021", "boost_stalker::X-00022", "boost_stalker::X-00023"};

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

  // Get the lookup and wait to populate (TODO: check for null?)
  hebi::Lookup lookup;
  sleep(2);
  lookup.printTable();

  // Get the group
  for (int i = 0; i < joint_names.size(); i++)
  {
    std::cout << "looking for: " << std::endl;
    std::cout << joint_names[i] << std::endl;
    std::cout << family_names[i] << std::endl;
  }
  std::unique_ptr<hebi::Group> group(lookup.getGroupFromNames(joint_names, family_names, 1000));
  if (!group)
  {
    ROS_INFO("Could not find modules on network! Quitting!");
    return -1;
  }
  // THIS IS A HACK to get around limited callback options for ROS subscribe call and the lack of a class for this node.
  group_g = group.get();

  std::cout << "Found modules!" << std::endl;
  ROS_INFO("Found modules!");

  // declare odometry message, required by robot state publisher.

  // Add an async feedback handler, sending feedback from one module to control a second
  // std::cout << "About to initialize /joint_states!" << std::endl;

  group->addFeedbackHandler(
    [&joint_pub, &joint_names](hebi::GroupFeedback* const fbk)->void
      {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "";

        // std::cout << "Initialized /joint_states!" << std::endl;

        // Add feedback:
        for (int i = 0; i < joint_names.size(); i++)
        {
          msg.name.push_back(joint_names[i].c_str());
          if ((*fbk)[i].actuatorFeedback().hasPosition())
            // Dirty, fix this later, deals with linear stage positioning
            if ((i == 1)) 
              msg.position.push_back((*fbk)[i].actuatorFeedback().getPosition()/STAGE_GEAR_RATIO);
            else 
              msg.position.push_back((*fbk)[i].actuatorFeedback().getPosition());
          if ((*fbk)[i].actuatorFeedback().hasVelocity())
            // Dirty, fix this later, deals with linear stage positioning
            if ((i == 1))
              msg.velocity.push_back((*fbk)[i].actuatorFeedback().getVelocity()/STAGE_GEAR_RATIO);
            else
              msg.velocity.push_back((*fbk)[i].actuatorFeedback().getVelocity());
          if ((*fbk)[i].actuatorFeedback().hasTorque())
            // Dirty, fix this later, deals with linear stage positioning
            if ((i == 1))
              msg.effort.push_back((*fbk)[i].actuatorFeedback().getTorque()*STAGE_GEAR_RATIO);
            else
              msg.effort.push_back((*fbk)[i].actuatorFeedback().getTorque());
        }

        joint_pub.publish(msg);
      });
  std::cout << "Added handler!" << std::endl;
  // Set the rate at which this gets called.  (TODO: make this a parameter)
  group->setFeedbackFrequencyHz(100);
  std::cout << "Set handler frequency!" << std::endl;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

   // Zero out all commands to the motors
  hebi::GroupCommand cmd(3);
  cmd[0].actuatorCommand().setPosition((nan(""), nan(""), nan("")));
  cmd[1].actuatorCommand().setVelocity((nan(""), nan(""), nan("")));
  cmd[2].actuatorCommand().setTorque((nan(""), nan(""), nan("")));
  group_g->sendCommand(cmd);


  ros::Subscriber sub_joint_command = n.subscribe("joint_commands", 1, commandCallback);
  ros::Subscriber sub_cmd_joy = n.subscribe("joy", 1, joyCallback);

  while (ros::ok())
  {
    //sleep(1);
    //ros::spinOnce();
    ros::spin();
  }

  // Stop the async callback before returning and deleting objects.
  group->clearFeedbackHandlers();

  sleep(1); // prevent segfaults? (TODO: needed?)

  return 0;
}
