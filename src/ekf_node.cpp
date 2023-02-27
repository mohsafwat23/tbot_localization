#include "tbot_localization/ekf_node.hpp"
#include "tbot_localization/ekf.hpp"

EKF_Node::EKF_Node(ros::NodeHandle *nh) 
{
    EKF();
    
    odom_subscriber = nh->subscribe("/odom", 10, &EKF_Node::odom_callback, this);

    imu_subscriber = nh->subscribe("/imu", 10, &EKF_Node::imu_callback, this);

    cam_subscriber = nh->subscribe("/cam_data", 10, &EKF_Node::cam_callback, this);

    
};

void EKF_Node::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //setOdom(msg->twist.twist.linear.x, msg->twist.twist.angular.z);
    odom_rob(0) = msg->twist.twist.linear.x;
    odom_rob(1) = msg->twist.twist.angular.z;
}

void EKF_Node::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Robot pose
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;

    theta_imu = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    // std::cout << theta_imu << "\n";

    // tnowIMU = double(msg->header.stamp.sec) + double(msg->header.stamp.nsec)*1e-9;

}

void EKF_Node::cam_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    nlandmarks = msg->layout.dim[0].size;       // number of landmarks observed (a.k.a number of rows)
    W = msg->layout.dim[1].size;                // number of measurements observed (a.k.a number of columns)
    cam_data = msg->data;
    cam_recieved = true;    

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_ekf");
    ros::NodeHandle nh;
    EKF_Node ekfN = EKF_Node(&nh);


    std::vector<double> lmP[7];   //landmark position
    std::vector<double> lmR[7];  //landmark orientation
    //Declare Parameters
    for(int i=0; i<7; i++)
    {
        if (nh.getParam("/params/ros__parameters/landmark" + std::to_string(i) + "/xyz", lmP[i]))
        {
            ROS_INFO("Got param");
        }
        else 
        {
            ROS_ERROR("Failed to get param 'my_param'");
        }

        if (nh.getParam("/params/ros__parameters/landmark" + std::to_string(i) + "/rpy", lmR[i]))
        {

        }
    }

    ros::Rate r(30.0);     // 100 hz

        
    int vec_size;
    ros::Publisher pose_marker_pub = nh.advertise<visualization_msgs::Marker>("/robot_marker_pose", 10);

    visualization_msgs::Marker pose_marker;

    while(ros::ok())
    {
        ros::spinOnce();
        ekfN.predict(1.0/30.0);
        //ekfN.updateIMU();
        
        //check if camera data is coming
        if(ekfN.cam_recieved)
        {
            vec_size = ekfN.cam_data.size();
        }
        else{
            vec_size = 0;
        }
        //check if the vector is valid and contains data
        if(vec_size == ekfN.nlandmarks*ekfN.W && vec_size > 0){
            //Eigen::Map<Eigen::MatrixXf> Landmarks(data.data(), nlandmarks, W);
            Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Landmarks(ekfN.cam_data.data(), ekfN.nlandmarks, ekfN.W);
            //Loop over each observation
            for(int i=0; i<ekfN.nlandmarks; i++)
            {
                int id_aruco = Landmarks(i,0);
                ekfN.updateCam(lmP[id_aruco], Landmarks(i,1), Landmarks(i,2));

            }
        
        }

        pose_marker.header.frame_id = "odom";
        pose_marker.header.stamp = ros::Time::now();
        pose_marker.ns = "tbot_marker";
        pose_marker.id = 1;
        pose_marker.type = visualization_msgs::Marker::ARROW;

        pose_marker.scale.x = 0.8;
        pose_marker.scale.y = 0.05;
        pose_marker.scale.z = 0.05;
        pose_marker.color.a = 1.0; // Don't forget to set the alpha!
        pose_marker.color.r = 1.0;
        pose_marker.color.g = 0.0;
        pose_marker.color.b = 0.0;
        pose_marker.pose.position.x = ekfN.getState()(0);
        pose_marker.pose.position.y = ekfN.getState()(1);
        pose_marker.pose.position.z = 0.0;
        pose_marker.pose.orientation.w = 1.0;
        pose_marker.pose.orientation.x = 0.0;
        pose_marker.pose.orientation.y = 0.0;
        pose_marker.pose.orientation.z = 0.0;
        pose_marker_pub.publish(pose_marker);
  
        // std::cout << ekfN.getState() << "\n";
        r.sleep();
    }


    return 0;
}