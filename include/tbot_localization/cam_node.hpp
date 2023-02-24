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
        // get raw image data
        void img_raw_callback(const sensor_msgs::Image::ConstPtr& msg);
        
        cv_bridge::CvImagePtr cv_ptr;

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        // Multiarray width
        const int W = 3;

        const std::string OPENCV_WINDOW = "Image window";


        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) 
                            << 921.9938565545156, 0.0,             480.5, 
                                0.0,            921.9938565545156, 360.5, 
                                0.0,              0.0,              1.0);

        cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 0.0,0.0,0.0,0.0,0.0);

        //ROS Node stuff
        ros::Subscriber img_raw_sub;
};





#endif