#include "ros/ros.h"
#include "automatic_exposure/AutomaticExposure.h"
#include "sensor_msgs/Image.h"
#include "dynamic_reconfigure/Config.h"
#include "dynamic_reconfigure/Reconfigure.h"
#include "dynamic_reconfigure/DoubleParameter.h"
#include <cstdlib>
#include <queue>

//**********CONSTANTS**************//
//Samples every rate-th pixel: [1, image height*width]
const int PIXEL_SAMPLING_RATE = 30;

/**Intensity goal as a percentage: [0, 100]
   Suggestions:
   40% for the field
   18% for indoors
**/
int INTENSITY_TARGET = 18;


//Deviation from INTENSITY_TARGET as a percentage [0, INTENSITY_TARGET]
const int INTENSITY_EPSILON = 3;


/**Number of times that camera calculates and adjusts exposure. A higher
   value will execute slower but probably give better values.
   Default is 3 and allows for ~1sec execution.
**/
const int ADJUSTMENT_ITERATIONS = 6;

//Location of node to be dynamically reconfigured
const std::string DUO_RECONFIG_NAME = "duo_node/set_parameters";

//Name of image topic that intensity data is pulled from
const std::string IMAGE_TOPIC = "duo3d_camera/left/image_rect";



//**********UTILITY FUNCTIONS**************//
//utility function
int toRGB(int percentage){
    return int(percentage / 100.0 * 255.0);
}

//utility function
int toPercent(int rgb){
    return int(rgb / 255.0 * 100.0);

}


//**********CHANGE CAMERA EXPOSURE THROUGH DYNAMIC RECONFIGURE**************//
bool changeExposure(int new_exposure){
        dynamic_reconfigure::ReconfigureRequest recfg_request;
        dynamic_reconfigure::ReconfigureResponse recfg_response;

        dynamic_reconfigure::DoubleParameter exp_param;
        dynamic_reconfigure::Config cfg;

        //Force value into bounds of [0, 100]
        new_exposure = std::max(std::min(100, new_exposure), 0);

        exp_param.name = "exposure";
        exp_param.value = double(new_exposure);
        cfg.doubles.push_back(exp_param);

        recfg_request.config = cfg;

        //Waits for dynamic reconfigure service and calls to change exposure
        if(ros::service::waitForService(DUO_RECONFIG_NAME, ros::Duration(20.0)) && ros::service::call(DUO_RECONFIG_NAME, recfg_request, recfg_response)){
            //successfully dynamically reconfigured
            ROS_INFO("Successfully changed exposure to %d", new_exposure);
            return true;
        }
        else{
            ROS_WARN("Could not complete dynamic_reconfigure call to change exposure.");
            return false;
        }
}


//**********CALCULATE NEW EXPOSURE FROM IMAGE DATA**************//

//Calculates average intensity from image as percent value [0, 100]
int getAverageIntensity(const sensor_msgs::Image::ConstPtr &image){

    //Calculate average image intensity
    int num_pixels = image->height * image->width;
    int intensity_sum = 0;
    for(int i = 0; i < num_pixels; i = i+PIXEL_SAMPLING_RATE){
        intensity_sum += image->data[i];
    }

    return toPercent( intensity_sum / (num_pixels / PIXEL_SAMPLING_RATE));
}

//*********RETRIEVE ONE IMAGE MESSAGE**********//
const sensor_msgs::Image::ConstPtr getOneImage(){

    boost::shared_ptr<sensor_msgs::Image const> image_ptr;

    image_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(IMAGE_TOPIC, ros::Duration(0.5));

    if(image_ptr == NULL) {
        ROS_ERROR("No image stream from camera. Hint: Check that camera is on and publishing to correct topic.");
        ros::shutdown();
    }
    else{
        return image_ptr;
    }
}


//**********SERVICE CALLBACK**************//
bool serviceCallback(automatic_exposure::AutomaticExposure::Request &request,
            automatic_exposure::AutomaticExposure::Response &response){
    INTENSITY_TARGET = request.intensity_target;

    ros::NodeHandle n;
    bool exposure_changed = true;
    double luminosity_ratio = 1.0;
    int new_intensity, new_exposure, old_intensity, old_exposure, intensity_diff;

    //Keeps track of last few exposures
    std::queue<int> oscillation_check;

    old_intensity = getAverageIntensity(getOneImage());
    new_intensity = old_intensity;
    n.getParam("duo_node/exposure", old_exposure);
    oscillation_check.push(old_exposure);


    //adjust exposure for ADJUSTMENT_ITERATIONS tries, or until target intensity is met.
    for(int i = 0; i < ADJUSTMENT_ITERATIONS && std::abs(INTENSITY_TARGET - new_intensity) > INTENSITY_EPSILON; ++i){
        ROS_INFO("i: %d", i);
        n.getParam("duo_node/exposure", old_exposure);
        old_intensity = new_intensity;

        intensity_diff = INTENSITY_TARGET - old_intensity;

        new_exposure = std::max(std::min(100, old_exposure + int(intensity_diff * luminosity_ratio)), 0);

        if(new_exposure == old_exposure){
            //checks if exposure is already min/maxxed out; no point continuing.
            if(new_exposure == 0 or new_exposure == 100){
                ROS_WARN("Cannot reach target. Camera exposure limited to %d%%.", new_exposure);
                return true;
            }

            //Set luminosity_ratio to something arbitrarily high to compensate for low scene reflectivity.
            luminosity_ratio = (std::rand() / double(RAND_MAX)) + 3.0;
            ROS_INFO("luminosity_ratio %f (where %d and %d are the same", luminosity_ratio, new_exposure, old_exposure);
            new_exposure = std::max(std::min(100, old_exposure + int(intensity_diff * luminosity_ratio)), 0);
        }

        //Check for oscillation if not in first iteration
        if(oscillation_check.size() > 1){
            ROS_INFO("queue front %d", oscillation_check.front());
            if(new_exposure == oscillation_check.front()){
                new_exposure = (new_exposure + old_exposure) / 2;
                ROS_INFO("found oscillation. new_exposure: %d", new_exposure);
            }
            oscillation_check.pop();
        }
        //Changes camera exposure.
        exposure_changed = changeExposure(new_exposure);
        oscillation_check.push(new_exposure);
        ROS_INFO("Oscillation queue check has size %d", int(oscillation_check.size()));
        new_intensity = getAverageIntensity(getOneImage());
        ROS_INFO("Exposure of %d%% yields intensity %d%%.", new_exposure, new_intensity);

        if(old_intensity == new_intensity){
            luminosity_ratio = old_exposure - new_exposure;
        }
        else{	luminosity_ratio = double(old_exposure - new_exposure)/double(old_intensity - new_intensity); }
        ROS_INFO("luminosity ratio %f", luminosity_ratio);
    }

    //Warns in case where scene is outside camera's range (or where more iterations of exposure adjustment are probably needed)
    if(std::abs(new_intensity - INTENSITY_TARGET) > INTENSITY_EPSILON){
        ROS_WARN("Exposure may not have adjusted correctly. Target: %d%% Actual: %d%%.", INTENSITY_TARGET, new_intensity);
    }
    else{ ROS_INFO("Exposure within target range."); }

    //returns whether the dynamic exposure call was successful
    return exposure_changed;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "automatic_exposure_server");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("automatic_exposure", &serviceCallback);
    ROS_INFO("Automatic Exposure service ready.");
    ros::spin();

    return 0;
}

