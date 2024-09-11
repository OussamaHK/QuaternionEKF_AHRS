/*
QuaternionEKF.h

Original Author: 
Jung Soon Jang
2006-08-31
University of Minnesota 
Aerospace Engineering and Mechanics 
Copyright 2011 Regents of the University of Minnesota. All rights reserved.

Updated to be a class, use Eigen, and compile as an Arduino library.
Added methods to set covariance and get gyro bias. Added initialization to 
estimated angles rather than assuming IMU is level. Added method to get
magnetic heading rather than just psi:
Brian R Taylor
brian.taylor@bolderflight.com
2017-12-14
Bolder Flight Systems
Copyright 2017 Bolder Flight Systems

Modified by Oussama Hadj Abdelkader for personal use, some equations were updated,
the covariance matrices were tuned, and the program was modified according to the use-case
hadjabdelkader.oussama@gmail.com
2024/09/11
The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.
*/

#ifndef QUATERNION_EKF_h
#define QUATERNION_EKF_h

#include "Arduino.h"
//#include "Eigen.h"
#include <ArduinoEigen.h>
//#include <Eigen/Dense>
#include <ArduinoEigenDense.h>
class QuaternionEKF {
  public:
    void setMagCF(uint8_t magCF);
    void setAccelCF(uint8_t accelCF);
    void setAccelCovariance(float cov);
    void setGyroCovariance(float cov);
    void setGyroBiasCovariance(float cov);
    void setHeadingCovariance(float cov);
    void update(float gx,float gy,float gz,float ax,float ay,float az,float mx, float my, float mz);
    float getPitch_rad();
    float getRoll_rad();
    float getYaw_rad();
    float getHeading_rad();
    float getGyroBiasX_rads();
    float getGyroBiasY_rads();
    float getGyroBiasZ_rads();
  private:
   // error covariance of accelerometers
    float var_a = 0.001f;//0.01f; //(0.1*g)^2        0.001
    // error covariance of magnetometer heading
    float var_psi = 0.01f;//0.01f; //(7*d2r)^2
    // error covariance of gyroscopes
    float var_g = 0.001f;//0.01f; (deg/s)^2         0.001
    // error covariance of gyroscopes bias
    float var_g_bias = 0.00001f;//0.00001f;  (deg/s)^2
    // estimated attitude
    float phi, theta, psi, SinTheta;
    // acceleration due to gravity
    const float G = 9.807f, G2 = 2.0f*9.807f;
    
    // initialization
    bool initialized = false;
    // initial heading
    float psi_Initial;
    // EKF state vector (quaternion q)
    Eigen::Matrix<float,7,1> q = Eigen::Matrix<float,7,1>::Zero();
    // timing
    float dt, tnow, tprev;
    // gyro integration (predicted rotation values)
    float gxc, gyc, gzc;
    // update frequency using magnetometer (not used)
//    uint8_t magCF_ = 1;
//    uint8_t magCount = 0;
    // update frequency using accelerometer (not used)
//    uint8_t accelCF_ = 1;
//    uint8_t accelCount = 0;
    // error, measurement, and process covariance matrices
    Eigen::Matrix<float,7,7> P_ = Eigen::Matrix<float,7,7>::Zero();
    Eigen::Matrix<float,7,7> Q = Eigen::Matrix<float,7,7>::Zero();
    Eigen::Matrix<float,3,3> R = Eigen::Matrix<float,3,3>::Zero();
    // gain matrix (with accelerator)
    Eigen::Matrix<float,7,3> K = Eigen::Matrix<float,7,3>::Zero();
    // state transition matrix
    Eigen::Matrix<float,7,7> F_ = Eigen::Matrix<float,7,7>::Zero();
    // measurement function (with accelerator)
    Eigen::Matrix<float,3,1> h_acc = Eigen::Matrix<float,3,1>::Zero();
    // Jacobian matrix of measurement function
    Eigen::Matrix<float,3,7> H_acc = Eigen::Matrix<float,3,7>::Zero();
    // heading based update matrices
    Eigen::Matrix<float,1,7> H_psi = Eigen::Matrix<float,1,7>::Zero();
    Eigen::Matrix<float,7,1> K_psi = Eigen::Matrix<float,7,1>::Zero();
    // tilt magnetic heading compensation using roll and pitch angle
    float Bxc, Byc;
    // limit yaw angle between -180 and 180
    float wraparound(float angle_);
    // limit heading angle between 0 and 360
    float boundAngle(float angle_);
};

#endif
