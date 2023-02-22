#ifndef EKF_h
#define EKF_h

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <vector>

class EKF {
    public:
        EKF();

        void predict();

        void updateCam(std::vector<double> landmark, const double dist, const double bearing);

        void updateIMU(double omega);

        void setEncoders(const float encl, const float encr);

        void setDt(const double tf, const double ti);

        Eigen::VectorXd getEncoders();
        
        Eigen::VectorXd getState();

        Eigen::MatrixXd getCovariance();


    private:

        double dt;           //time
        float Width;
        Eigen::Vector2d u_enc;    
        Eigen::Vector3d state;    //x,y,theta
        Eigen::Matrix3d sigma;
        
        // How uncertain are you with model; how far is x_hat from model and x
        Eigen::Matrix2d Q;       //process noise covariance

        // How uncertain are you with measurement; how far is x_hat from measurement and x
        // Todo: measurement noise changes with distance from landmark
        Eigen::Matrix2d R_cam;      //Measurement noise for Camera
        Eigen::Matrix3d I; 

        // Measurement noise for Gyroscope
        float R_gyr;
        
        // motion model Jacobian 
        Eigen::Matrix3d J_fx; 
        
        // controls inputs Jacobian
        Eigen::Matrix<double, 3,2> J_fu; 

        // Camera observation model Jacobian
        Eigen::Matrix<double, 2,3> J_H; 

        // IMU Observation model
        Eigen::Matrix<double, 1,3> H_gyr;

        Eigen::Vector2d measurement;    //measurement
        Eigen::Vector2d measurement_hat;    //predicted measurement

        //Eigen::Vector2d landmark;

        // Kalman Gains
        Eigen::Matrix<double, 3,2> K_cam; //= Eigen::Matrix<double, 3,2>::Zero(); //Kalman gain
        Eigen::Matrix<double, 3,1> K_gyr;


        // Camera measurement noise covariance
        Eigen::Matrix2d S_obs_cam; // = Eigen::Matrix2d::Zero(); //measurement noise covariance

        // Camera measurement noise covariance
        double S_obs_gyr; // = Eigen::Matrix2d::Zero(); //measurement noise covariance

};

#endif