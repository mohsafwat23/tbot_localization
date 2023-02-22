#include "tbot_localization/ekf.hpp"

EKF::EKF()
{
    Width = 0.4;
    dt = 0.01;
    state = Eigen::Vector3d::Zero();
    u_enc = Eigen::Vector2d::Zero();
    sigma = Eigen::Matrix3d::Zero();
    //landmark = Eigen::Vector2d::Zero();

    // Process Noise Covariance Matrix
    Q = (Eigen::Matrix2d() << 0.1*0.1,0.0,0.0,0.1*0.1).finished();

    // Camera Measurement Noise Covariance
    R_cam = (Eigen::Matrix2d() << 0.2*0.2,0.0,0.0,0.1*0.1).finished();

    // Gyro Noise Covariance
    R_gyr = 0.005;

    I = Eigen::Matrix3d::Identity();
}

void EKF::predict()
{
    //prior motion model
    //x position
    state(0) += cos(state(2))*dt*((u_enc(0)+u_enc(1))/2.0);
    //y position
    state(1) += sin(state(2))*dt*((u_enc(0)+u_enc(1))/2.0);
    //theta
    state(2) += dt*((u_enc(1)-u_enc(0))/Width);

    //this handles angle wrapping
    state(2) = atan2(sin(state(2)), cos(state(2)));

    //motion model Jacobian
    J_fx << 1.0, 0.0, -sin(state(2))*dt*((u_enc(0)+u_enc(1))/2.0),
            0.0, 1.0,  cos(state(2))*dt*((u_enc(0)+u_enc(1))/2.0),
            0.0, 0.0, 1.0;

    //control model Jacobian 
    J_fu << cos(state(2))*dt/2.0, cos(state(2))*dt/2.0,
            sin(state(2))*dt/2.0, sin(state(2))*dt/2.0,
            -dt/Width                ,                  dt/Width;

    //prior covariance 
    sigma = J_fx*sigma*J_fx.transpose() + J_fu*Q*J_fu.transpose();
}

void EKF::updateCam(std::vector<double> landmark, const double dist, const double bearing)
{
    // Measurement Model:
    // dist = sqrt((x-xlandmark)^2 + (y-ylandmark)^2)
    // alpha = tan^-1((ylandmark - y)/(xlandmark - x)) - theta

    measurement(0) = dist;
    measurement(1) = bearing;
    measurement_hat(0) = sqrt(pow((state(0) - landmark[0]),2) + pow((state(1) - landmark[1]),2));
    measurement_hat(1) = atan2((landmark[1]-state(1)), (landmark[0]-state(0))) - state(2); 
    
    //this handles angle wrapping
    measurement_hat(1) = atan2(sin(measurement_hat(1)), cos(measurement_hat(1)));
    
    
    //Observation jacobian
    J_H <<  (state(0)-landmark[0])/measurement_hat(0), (state(1)- landmark[1])/measurement_hat(0), 0.0,
            (-state(1)+landmark[1])/pow(measurement_hat(0),2),             (state(0)-landmark[0])/pow(measurement_hat(0),2),              -1.0;

    //measurement noise covariance
    S_obs_cam = J_H*sigma*J_H.transpose() + R_cam;

    //Kalman gain
    K_cam = sigma*J_H.transpose()*(S_obs_cam).inverse();

    //update state
    state = state + K_cam*(measurement - measurement_hat);

    //update covarance
    sigma = (I - K_cam*J_H)*sigma;

}

void EKF::updateIMU(double omega)
{
    // double omega_hat = state(2)*dt;

    // Measurement Model:
    // [omega] = [0 0 1/dt]*[x y theta]^T

    H_gyr << 0, 0, 1/dt;

    //measurement noise covariance: CHECK THIS
    S_obs_gyr = H_gyr*sigma*H_gyr.transpose() + R_gyr;

    // Kalman gain
    K_gyr = sigma*H_gyr.transpose()*(1/S_obs_gyr);

    //update state
    state = state + K_gyr*(omega - (H_gyr*state)(0));

    //update covarance
    sigma = (I - K_gyr*H_gyr)*sigma;
}


void EKF::setEncoders(const float encl, const float encr)
{
    u_enc(0) = encl;
    u_enc(1) = encr;
}

void EKF::setDt(const double tf, const double ti)
{
    dt = tf - ti;
    if (dt > 1.0){
        dt = 0.0;
    }
}

Eigen::VectorXd EKF::getEncoders()
{
    return u_enc;
}

Eigen::VectorXd EKF::getState()
{
    return state;
}

Eigen::MatrixXd EKF::getCovariance()
{
    return sigma;
}