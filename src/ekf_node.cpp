#include "tbot_localization/ekf_node.hpp"
#include "tbot_localization/ekf.hpp"

EKF_Node::EKF_Node(ros::NodeHandle *nh) 
{
    
    odom_subscriber = nh->subscribe("/odom", 10, &EKF_Node::odom_callback, this);

    imu_subscriber = nh->subscribe("/imu_data", 10, &EKF_Node::imu_callback, this);

    //cam_subscriber = nh->subscribe("/cam_data", 10, &EKF_Node::cam_callback, this);
    
};

void EKF_Node::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{

}

void EKF_Node::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{

}

void cam_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{

}



int main()
{
    return 0;
}