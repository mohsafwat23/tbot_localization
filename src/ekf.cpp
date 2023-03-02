#include "tbot_localization/ekf.hpp"

EKF::EKF()
{
    Width = 0.4;
    dt = 0.01;
    state = Eigen::Vector3d::Zero();
    odom_rob = Eigen::Vector2d::Zero();
    sigma = Eigen::Matrix3d::Zero();
    //landmark = Eigen::Vector2d::Zero();

    // Process Noise Covariance Matrix
    Q = (Eigen::Matrix2d() << 0.1*0.1,0.0,0.0,0.1*0.1).finished();

    // Camera Measurement Noise Covariance
    R_cam = (Eigen::Matrix2d() << 0.2*0.2,0.0,0.0,0.8*0.8).finished();

    // Gyro Noise Covariance
    // R_imu = 0.005;
    R_gyro = 0.005;


    I = Eigen::Matrix3d::Identity();
}

void EKF::predict(const double dt_loop)
{
    dt = dt_loop;
    //prior motion model
    double v = odom_rob(0);
    double omega = odom_rob(1);

    //x position
    state(0) += v*dt*cos(state(2));             // x_t+1 = x_t + v*dt*cos(theta)
    //y position
    state(1) += v*dt*sin(state(2));             // y_t+1 = y_t + v*dt*sin(theta)
    //theta
    state(2) += omega*dt;                       // theta_t_1 = theta_t + omega*dt

    //this handles angle wrapping
    state(2) = atan2(sin(state(2)), cos(state(2)));

    //motion model Jacobian
    J_fx << 1.0, 0.0, -v*dt*sin(state(2))*omega,            //     dx_{t+1}/dx_t       dx_{t+1}/dy_t      dx_{t+1}/dtheta_t
            0.0, 1.0,  v*dt*cos(state(2))*omega,            //     dy_{t+1}/dy_t       dy_{t+1}/dy_t      dy_{t+1}/dtheta_t
            0.0, 0.0,                       1.0;            // dtheta_{t+1}/dx_t   dtheta_{t+1}/dy_t  dtheta_{t+1}/dtheta_t

    //control model Jacobian 
    J_fu << cos(state(2))*dt, 0.0,                          //     dx_{t+1}/dv_t           dx_{t+1}/domega_t
            sin(state(2))*dt, 0.0,                          //     dy_{t+1}/dv_t           dy_{t+1}/domega_t
            0.0             ,  dt;                          //     dtheta_{t+1}/dv_t       dtheta_{t+1}/domega_t

    //prior covariance 
    sigma = J_fx*sigma*J_fx.transpose() + J_fu*Q*J_fu.transpose();
}

void EKF::updateCam(std::vector<double> landmark, const double dist, const double bearing)
{
    // Measurement Model:
    // dist = sqrt((x-xlandmark)^2 + (y-ylandmark)^2 + (z - zlandmark)^2)
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


    // Covariance increases the further you are from the april tag
    R_cam << measurement_hat(0)*0.05, 0.0, 0.0, measurement_hat(0)*0.1;
    //measurement noise covariance
    S_obs_cam = J_H*sigma*J_H.transpose() + R_cam;

    //Kalman gain
    K_cam = sigma*J_H.transpose()*(S_obs_cam).inverse();

    //update state
    state = state + K_cam*(measurement - measurement_hat);

    //update covarance
    sigma = (I - K_cam*J_H)*sigma;

}

// void EKF::updateIMU()
// {
//     // Measurement Model:
//     // [theta] = [0 0 1]*[x y theta]^T

//     H_imu << 0.0, 0.0, 1.0;
    

//     //measurement noise covariance: CHECK THIS
//     S_obs_imu = H_imu*sigma*H_imu.transpose() + R_imu;


//     // Kalman gain
//     K_imu = sigma*H_imu.transpose()*(1/S_obs_imu);

//     //update state
//     state = state + K_imu*(theta_imu - (H_imu*state)(0));

//     //update covarance
//     sigma = (I - K_imu*H_imu)*sigma;
// }

void EKF::updateGyro(const double dt_loop)
{
    // Measurement Model:
    // [theta] = [0 0 dt]*[x y theta]^T

    H_gyro << 0.0, 0.0, dt_loop;
    

    //measurement noise covariance: CHECK THIS
    S_obs_gyro = H_gyro*sigma*H_gyro.transpose() + R_gyro;


    // Kalman gain
    K_gyro = sigma*H_gyro.transpose()*(1/S_obs_gyro);

    //update state
    state = state + K_gyro*(theta_imu - (H_gyro*state)(0));

    //update covarance
    sigma = (I - K_gyro*H_gyro)*sigma;
}


void EKF::setOdom(const float v_rob, const float omega_rob)
{
    odom_rob(0) = v_rob;
    odom_rob(1) = omega_rob;
}

void EKF::setDt(const double tf, const double ti)
{
    dt = tf - ti;
    if (dt > 1.0){
        dt = 0.0;
    }
}

// Eigen::VectorXd EKF::getEncoders()
// {
//     return u_enc;
// }

Eigen::VectorXd EKF::getState()
{
    return state;
}

Eigen::MatrixXd EKF::getCovariance()
{
    return sigma;
}