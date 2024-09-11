/*
QuaternionEKF.cpp

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

Modified by Oussama Hadj Abdelkader for personal use, 
the covariance matrices are tuned, and the program is modified according to the use-case
hadjabdelkader.oussama@gmail.com
2024/09/11
The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.
*/

#include "Arduino.h"
#include "QuaternionEKF.h"

// sets the update step frequency using magnetometer (not used)
//void QuaternionEKF::setMagCF(uint8_t magCF) {
//  magCF_ = magCF;
//}
//void QuaternionEKF::setAccelCF(uint8_t accelCF) {
//  accelCF_ = accelCF;
//}

// sets the accelerometer covariance, (m/s/s)^2
void QuaternionEKF::setAccelCovariance(float cov) {
  var_a = cov;
}

// sets the magnetometer heading covariance, (rad)^2
void QuaternionEKF::setHeadingCovariance(float cov) {
  var_psi = cov;
}

// sets the gyroscope covariance, (rad/s)^2
void QuaternionEKF::setGyroCovariance(float cov) {
  var_g = cov;
}

// sets the gyroscope bias covariance, (rad/s)^2
void QuaternionEKF::setGyroBiasCovariance(float cov) {
  var_g_bias = cov;
}

// EKF prediction and update in one function
// gyroscope gx, gy, gz inputs in rad/s
// accelerometer ax, ay, az inputs in m/s/s
// magnetometer mx, my, mz inputs in microTeslas
void QuaternionEKF::update(float gx,float gy,float gz,float ax,float ay,float az,float mx, float my, float mz) {
  if (!initialized) {
    // set the time
    tprev = (float) micros()/1000000.0f;
    // initial attitude and heading
    theta = atan2(-ax,sqrtf((ay*ay)+(az*az)));
    //theta = asinf(ax/G);
    phi = atan2(ay,az);
    //phi = asinf(-ay/(G*cosf(theta)));
    // magnetic heading correction due to roll and pitch angle
    Bxc = mx*cosf(theta) + (my*sinf(phi) + mz*cosf(phi))*sinf(theta);
    Byc = my*cosf(phi) - mz*sinf(phi);
    //Bxc = mx*cosf(theta) + mz*sinf(theta);
    //Byc = my*cosf(phi) + mx*sinf(phi)*sinf(theta) - mz*sinf(phi)*cosf(theta);
    // finding initial heading
    psi = atanf(Byc/Bxc);
//    if (-Byc > 0) {
//      psi = PI/2.0f - atanf(Bxc/-Byc);
//    } else {
//      psi = 3.0f*PI/2.0f - atanf(Bxc/-Byc);
//    }
    psi = wraparound(psi);
    psi_Initial = psi;

    // euler to quaternion transformation
    q(0,0) = cosf(psi/2.0f)*cosf(theta/2.0f)*cosf(phi/2.0f) + sinf(psi/2.0f)*sinf(theta/2.0f)*sinf(phi/2.0f);  
    q(1,0) = cosf(psi/2.0f)*cosf(theta/2.0f)*sinf(phi/2.0f) - sinf(psi/2.0f)*sinf(theta/2.0f)*cosf(phi/2.0f);
    q(2,0) = cosf(psi/2.0f)*sinf(theta/2.0f)*cosf(phi/2.0f) + sinf(psi/2.0f)*cosf(theta/2.0f)*sinf(phi/2.0f);  
    q(3,0) = sinf(psi/2.0f)*cosf(theta/2.0f)*cosf(phi/2.0f) - cosf(psi/2.0f)*sinf(theta/2.0f)*sinf(phi/2.0f);
    // initialization of error, measurement, and process covariance matrices
    P_(0,0)=P_(1,1)=P_(2,2)=P_(3,3)=0.1f; P_(4,4)=P_(5,5)=P_(6,6)=0.1f;
    Q(0,0)=Q(1,1)=Q(2,2)=Q(3,3)=var_g; 
    Q(4,4)=Q(5,5)=Q(6,6)=var_g_bias;
    R(0,0)=R(1,1)=R(2,2)=var_a;
    initialized = true;
  } else {
    // get the time step (using sensors frequency doesn't work because of jitter)
    tnow = (float) micros()/1000000.0f;
    dt = 0.5f * (tnow - tprev);// time step
    tprev = tnow;
    // gyro integration
    gxc = (gx - q(4,0))*dt;
    gyc = (gy - q(5,0))*dt;
    gzc = (gz - q(6,0))*dt;
    // state transition matrix
                              F_(0,1) = -gxc;         F_(0,2) = -gyc;         F_(0,3) = -gzc;
    F_(1,0) =  gxc;                                   F_(1,2) =  gzc;         F_(1,3) = -gyc;
    F_(2,0) =  gyc;          F_(2,1) = -gzc;                                  F_(2,3) =  gxc;
    F_(3,0) =  gzc;          F_(3,1) =  gyc;         F_(3,2) = -gxc;
    
    F_(0,4) = q(1,0)*dt;   F_(0,5) = q(2,0)*dt;  F_(0,6) = q(3,0)*dt;
    F_(1,4) = -q(0,0)*dt;  F_(1,5) = q(3,0)*dt;  F_(1,6) = -F_(0,5);
    F_(2,4) = -F_(1,5);   F_(2,5) = F_(1,4);   F_(2,6) = F_(0,4);
    F_(3,4) = F_(0,5);    F_(3,5) = -F_(0,4);  F_(3,6) = F_(1,4);
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Extended Kalman filter: prediction step using gyroscope data
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // propagation of quaternion using gyro measurement at a given sampling interval dt
    q(0,0) += -gxc*q(1,0) - gyc*q(2,0) - gzc*q(3,0);
    q(1,0) +=  gxc*q(0,0) - gyc*q(3,0) + gzc*q(2,0);
    q(2,0) +=  gxc*q(3,0) + gyc*q(0,0) - gzc*q(1,0);
    q(3,0) += -gxc*q(2,0) + gyc*q(1,0) + gzc*q(0,0); 
    // error covariance propagation: P = F_*P*F_' + Q
    P_ = F_*P_*F_.transpose() + Q;
    //if (accelCount == accelCF_) { (not used)
      //accelCount = 0;
      // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // Extended Kalman filter: correction step for pitch and roll using accelerometer measurements
      // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // nonlinear measurement equation of h_acc(x)(computing gravity in body frame by rotating the gravity in earth frame)
      h_acc(0,0) = -G2*(q(1,0)*q(3,0)-q(0,0)*q(2,0));
      h_acc(1,0) = -G2*(q(0,0)*q(1,0)+q(2,0)*q(3,0));
      h_acc(2,0) = -G*(q(0,0)*q(0,0)-q(1,0)*q(1,0)-q(2,0)*q(2,0)+q(3,0)*q(3,0));
      // compute Jacobian matrix of h_acc(x)
      H_acc(0,0) = G2*q(2,0);     H_acc(0,1) =-G2*q(3,0);           H_acc(0,2) = G2*q(0,0);     H_acc(0,3) = -G2*q(1,0);
      H_acc(1,0) = H_acc(0,3);        H_acc(1,1) =-H_acc(0,2);              H_acc(1,2) = H_acc(0,1);        H_acc(1,3) = -H_acc(0,0);
      H_acc(2,0) =-H_acc(0,2);        H_acc(2,1) =-H_acc(0,3);              H_acc(2,2) = H_acc(0,0);        H_acc(2,3) =  H_acc(0,1);
      // gain matrix K = P_*H_acc'*(H_acc*P_*H_acc' + R)^-1
      K = P_*H_acc.transpose()*(H_acc*P_*H_acc.transpose() + R).inverse();
      
      // state update
      for(size_t i=0; i < 7; i++)
      {
        q(i,0) += K(i,0)*(ax - h_acc(0,0))
               +  K(i,1)*(ay - h_acc(1,0))
               +  K(i,2)*(az - h_acc(2,0));
      }
      // error covariance matrix update P_ = (I - K*H_acc)*P_
      P_ = (Eigen::Matrix<float,7,7>::Identity() - K*H_acc)*P_;
    //}
    //if(magCount == (magCF_)) { // (not used)
      //magCount = 0;
      // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // second stage kalman filter update to estimate the heading angle (from magnetometer data)
      // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // magnetic heading correction due to roll and pitch angle
      Bxc = mx*cosf(theta) + (my*sinf(phi) + mz*cosf(phi))*sinf(theta);
      Byc = my*cosf(phi) - mz*sinf(phi);
      //Bxc = mx*cosf(theta) + mz*sinf(theta);
      //Byc = my*cosf(phi) + mx*sinf(phi)*sinf(theta) - mz*sinf(phi)*cosf(theta);
      // normalization of quaternion,||q||^2 = 1
      for(size_t i=0; i < 4; i++) {
        q(i,0) = q(i,0)*1.0/sqrtf(q(0,0)*q(0,0)+q(1,0)*q(1,0)+q(2,0)*q(2,0)+q(3,0)*q(3,0));
      }
      // Jacobian
      H_psi(0,0) = q(3,0)*(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0)))*2.0f/(2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))*2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))+(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0)))*(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0))));
      H_psi(0,1) = q(2,0)*(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0)))*2.0f/(2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))*2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))+(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0)))*(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0))));
      H_psi(0,2) = q(1,0)*(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0)))*2.0f/(2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))*2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))+(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0)))*(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0))))+2.0f*q(2,0)*2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))*2.0f/(2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))*2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))+(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0)))*(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0))));
      H_psi(0,3) = q(0,0)*(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0)))*2.0f/(2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))*2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))+(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0)))*(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0))))+2.0f*q(3,0)*2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))*2.0f/(2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))*2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0))+(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0)))*(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0))));

      // gain matrix K_psi = P_*H_psi'*(H_psi*P_*H_psi' + R_psi)^-1
      K_psi = P_*H_psi.transpose()*1.0f/(H_psi*P_*H_psi.transpose() + var_psi);

      // state update
      psi = atan2f(2.0f*(q(1,0)*q(2,0)+q(0,0)*q(3,0)),(1.0f-2.0f*(q(2,0)*q(2,0)+q(3,0)*q(3,0))));
      for(size_t i=0; i < 7; i++) {
        q(i,0) += K_psi(i,0)*wraparound(atan2f(-Byc,Bxc)  - psi);
      }

      // error covariance matrix update P_ = (I - K_psi*H_psi)*P_
      P_ = (Eigen::Matrix<float,7,7>::Identity() - K_psi*H_psi)*P_;
    }
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // normalization of quaternion,||q||^2 = 1
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    for(size_t i=0; i < 4; i++) {
      q(i,0) = q(i,0)*1.0f/sqrtf(q(0,0)*q(0,0)+q(1,0)*q(1,0)+q(2,0)*q(2,0)+q(3,0)*q(3,0));
    } 
    //quaternion to euler angles transformation
    SinTheta = 2.0f*((q(0,0)*q(2,0))-(q(3,0)*q(1,0)));
    // limit Sin(theta) between -1 and 1 to avoid NaN values
    if (SinTheta >1){SinTheta = 1;}
    if (SinTheta <-1){SinTheta = -1;}
    theta = asinf(SinTheta);
    phi = atan2f(2.0f*((q(0,0)*q(1,0))+(q(2,0)*q(3,0))),1.0f-(2.0f*((q(1,0)*q(1,0))+(q(2,0)*q(2,0)))));
    psi = atan2f(2.0f*((q(1,0)*q(2,0))+(q(0,0)*q(3,0))),1.0f-(2.0f*((q(2,0)*q(2,0))+(q(3,0)*q(3,0)))));
    
    //magCount++; //(not used)
    //accelCount++;
  //}
}

// returns the pitch angle, rad
float QuaternionEKF::getPitch_rad() {
  return theta;
}

// returns the roll angle, rad
float QuaternionEKF::getRoll_rad() {
  return phi;
}

// returns the yaw angle, rad
float QuaternionEKF::getYaw_rad() {
  return wraparound(psi-psi_Initial);
}

// returns the heading angle, rad
float QuaternionEKF::getHeading_rad() {
  return boundAngle(psi);
}

// returns the gyro bias estimate in the x direction, rad/s
float QuaternionEKF::getGyroBiasX_rads() {
  return q(4,0);
}

// returns the gyro bias estimate in the y direction, rad/s
float QuaternionEKF::getGyroBiasY_rads() {
  return q(5,0);
}

// returns the gyro bias estimate in the z direction, rad/s
float QuaternionEKF::getGyroBiasZ_rads() {
  return q(6,0);
}
float QuaternionEKF::wraparound(float angle_){
// limit yaw angle between -180 and 180
  if(angle_ >  PI) angle_ -= (PI*2.0f);
  if(angle_ < -PI) angle_+= (PI*2.0f);
  return angle_;
}

// limit heading angle between 0 and 360 
float QuaternionEKF::boundAngle(float angle_){
  angle_ = fmod(angle_,2.0f*PI);
  if (angle_ < 0)
    angle_ += 2.0f*PI;
  return angle_;
}
