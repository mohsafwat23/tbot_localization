#ifndef CAM_NODE_h
#define CAM_NODE_h

#include <chrono>
#include <memory>
#include <functional>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32MultiArray.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/opencv.hpp"


class Cam_Node {
    public:
        Cam_Node(ros::NodeHandle *nh);

        //April Tag length
        double markerLength;
    
    private:

        const float Zoffset = 0.06;               // relative to coordinate frame of april tag

        const float Yoffset = 0.053;               // relative to coordinate frame of april tag

        // get raw image data
        void img_raw_callback(const sensor_msgs::Image::ConstPtr& msg);
        
        cv_bridge::CvImagePtr cv_ptr;

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        // Multiarray width
        const int W = 3;

        const std::string OPENCV_WINDOW = "Image window";


        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) 
                            << 909.74712849, 0.0,             661.62344023, 
                                0.0,            907.00789333, 390.19897635, 
                                0.0,              0.0,              1.0);

        cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 0.15587604, -0.35818478, 0.01418107, 0.00112367, 0.222139);

        //ROS Node stuff
        ros::Publisher cam_data_pub;
        ros::Subscriber img_raw_sub;
};





#endif