#ifndef EKF_NODE_h
#define EKF_NODE_h

#include "tbot_localization/ekf.hpp"
#include <chrono>
#include <memory>
#include <functional>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Imu.h"



class EKF_Node: public EKF {
    public:
        EKF_Node(ros::NodeHandle *nh);

        double tnowIMU;

        double tprevIMU;

        double tnowOdom;

        double tprevOdom;

        bool cam_recieved = false;

        // Number of Landmarks observed
        int nlandmarks;

        int W;

        std::vector<float> cam_data;


    private:
        // Call ekf class
        // EKF ekf;

        // Encoder Data
        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg); 

        // IMU Data
        void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

        // Observation data
        void cam_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

        // Publish state estimate
        void publisher_callback();

        //Landmarks from YAML
        // std::vector<double> lmP[7];   //landmark position
        // std::vector<double> lmR[7];  //landmark orientation

        //ROS Node stuff
        ros::Subscriber odom_subscriber;
        ros::Subscriber imu_subscriber;
        ros::Subscriber cam_subscriber;

        //Subscribe to encoder data
};

#endif
