#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <log4cxx/logger.h>

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        pub_ = n_.advertise<std_msgs::Float32>("/load_cell_pounds", 1);
        sub_ = n_.subscribe("/load_cell", 1, &SubscribeAndPublish::callback, this);
    }

    void callback(const std_msgs::Int16::ConstPtr& msg)
    {
       //ROS_INFO_STREAM("I heard: [%s]" << msg->data);
       std_msgs::Float32 pounds;
       pounds.data = (float((msg->data)-1810)/75.0);  //approximately 91 ticks per pound  //originally 1290 and 91
       pub_.publish(pounds);
    }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ticksToPounds2");
  ros::NodeHandle n;
  SubscribeAndPublish SAPObject;
  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
  ros::spin();

  return 0;
}
