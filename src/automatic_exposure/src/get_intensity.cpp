#include "ros/ros.h"
#include "sensor_msgs/Image.h"

//Quick utility node to output average intensity.

//Topic name
const std::string IMAGE_TOPIC = "duo3d_camera/left/image_rect";

//Samples every rate-th pixel: [1, image height*width]
const int PIXEL_SAMPLING_RATE = 3;
	

int getAverageIntensity(const sensor_msgs::Image::ConstPtr &image){

        //Calculate average image intensity
        int num_pixels = image->height * image->width;
        int intensity_sum = 0;
        for(int i = 0; i < num_pixels; i = i+PIXEL_SAMPLING_RATE){
                intensity_sum += image->data[i];
        }

        return int( intensity_sum / (num_pixels / PIXEL_SAMPLING_RATE));
}


int main(int argc, char ** argv){
	ros::init(argc, argv, "get_intensity");
	ros::NodeHandle nh;

	const sensor_msgs::Image::ConstPtr img = ros::topic::waitForMessage<sensor_msgs::Image>(IMAGE_TOPIC, ros::Duration(1.0));
	if(!img)
	{
		ROS_WARN("Could not pull image.");
	}
	else{
		int average = getAverageIntensity(img);
		ROS_INFO("Average intensity: %d%% (%d of 255)", int(average / 255.0 * 100.0), average);
	}

	ros::spinOnce();
}

