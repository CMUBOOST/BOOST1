#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  if (argc < 7) {
      // Tell the user how to run the program
      std::cerr << "This node is only designed for operating the arms" << std::endl;
      std::cerr << "Usage: " << "POS 21, " << "POS 22, " << "POS 23, " << "VEL 21, " << "VEL 22, " << "VEL 23, " << std::endl;
      /* "Usage messages" are a conventional way of telling the user
       * how to run a program if they enter the command incorrectly.
       */
      return 1;
  }

  std::cout << atof(argv[1]) << " " << atof(argv[2]) << " " << atof(argv[3]) << " " << atof(argv[4]) << " " << atof(argv[5]) << " " << atof(argv[6]) << std::endl;

  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<sensor_msgs::JointState>("/joint_commands", 1);
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    sensor_msgs::JointState jointActuate;
    jointActuate.header.stamp = ros::Time::now();
    jointActuate.header.frame_id = "";
    jointActuate.position.push_back(NAN); //float(*argv[1]-'0'));
    jointActuate.position.push_back(NAN); //float(*argv[2]-'0'));
    jointActuate.position.push_back(NAN); //float(*argv[3]-'0'));
    jointActuate.position.push_back(NAN); //float(*argv[3]-'0'));
    jointActuate.position.push_back(atof(argv[1])); //float(*argv[1]-'0'));
    jointActuate.position.push_back(atof(argv[2])); //float(*argv[2]-'0'));
    jointActuate.position.push_back(atof(argv[3])); //float(*argv[3]-'0'));
    jointActuate.velocity.push_back(NAN); //float(*argv[4]-'0'));
    jointActuate.velocity.push_back(NAN); //float(*argv[5]-'0'));
    jointActuate.velocity.push_back(NAN); //float(*argv[6]-'0'));
    jointActuate.velocity.push_back(NAN); //float(*argv[6]-'0'));    
    jointActuate.velocity.push_back(atof(argv[4])); //float(*argv[4]-'0'));
    jointActuate.velocity.push_back(atof(argv[5])); //float(*argv[5]-'0'));
    jointActuate.velocity.push_back(atof(argv[6])); //float(*argv[6]-'0'));

    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(jointActuate);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}