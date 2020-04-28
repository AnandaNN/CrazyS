/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/transform_datatypes.h"
#include "rotors_control/Matrix3x3.h"
#include "rotors_control/Quaternion.h"

#include "rotors_control/crazyflie_onboard_controller_v2.h"

#define SAMPLING_TIME_ATTITUDE_CONTROLLER         0.002 /* SAMPLING TIME ATTITUDE CONTROLLER [s] */
#define SAMPLING_TIME_RATE_CONTROLLER             0.002 /* SAMPLING TIME RATE CONTROLLER [s] */

double limit_integ(double integ, double limit) {
    if( integ > limit ) integ = limit;
    if( integ < -limit ) integ = -limit;
    return integ;
}

namespace rotors_control{

CrazyflieOnboardController::CrazyflieOnboardController()
    : counter_(false),
    delta_psi_ki_(0),
    p_command_(0),
    q_command_(0),
    p_command_ki_(0),
    q_command_ki_(0),
    p_command_kd_(0),
    q_command_kd_(0),
    p_integ_(0),
    q_integ_(0),
    p_prev_error_(0),
    q_prev_error_(0){

        ROS_INFO("Initializing all pid controllers");

              //TODO: get parameters from configuration manager instead
        pidInit(&pidRollRate,  0, PID_ROLL_RATE_KP,  PID_ROLL_RATE_KI,  PID_ROLL_RATE_KD,
            ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
        pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
            ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
        pidInit(&pidYawRate,   0, PID_YAW_RATE_KP,   PID_YAW_RATE_KI,   PID_YAW_RATE_KD,
            ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);

        pidSetIntegralLimit(&pidRollRate,  PID_ROLL_RATE_INTEGRATION_LIMIT);
        pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
        pidSetIntegralLimit(&pidYawRate,   PID_YAW_RATE_INTEGRATION_LIMIT);

        pidInit(&pidRoll,  0, PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  ATTITUDE_UPDATE_DT,
            ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
        pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, ATTITUDE_UPDATE_DT,
            ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);

        pidSetIntegralLimit(&pidRoll,  PID_ROLL_INTEGRATION_LIMIT);
        pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
        

}

void CrazyflieOnboardController::ResetPIDControllers() {

    pidReset(&pidRoll);
    pidReset(&pidPitch);
    pidReset(&pidRollRate);
    pidReset(&pidPitchRate);
    pidReset(&pidYawRate);

}

CrazyflieOnboardController::~CrazyflieOnboardController() {}

// Make a copy of control signals and get them private
void CrazyflieOnboardController::SetControlSignals(const control_s& control_t) {

    control_t_private_ = control_t;
}

// Make a copy of the drone state and get it private
void CrazyflieOnboardController::SetDroneState(const state_s& state_t) {

    state_t_private_ = state_t;
}

// Set the controller gains as local global variables
void CrazyflieOnboardController::SetControllerGains(PositionControllerParameters& controller_parameters_) {

      attitude_gain_kp_private_ = Eigen::Vector2f(controller_parameters_.attitude_gain_kp_.x(), controller_parameters_.attitude_gain_kp_.y());
      attitude_gain_ki_private_ = Eigen::Vector2f(controller_parameters_.attitude_gain_ki_.x(), controller_parameters_.attitude_gain_ki_.y());
      attitude_gain_kd_private_ = Eigen::Vector2f(controller_parameters_.attitude_gain_kd_.x(), controller_parameters_.attitude_gain_kd_.y());
      attitude_integration_limit_ = Eigen::Vector2f(controller_parameters_.attitude_integration_limit_.x(), controller_parameters_.attitude_integration_limit_.y());

      rate_gain_kp_private_ = Eigen::Vector3f(controller_parameters_.rate_gain_kp_.x(), controller_parameters_.rate_gain_kp_.y(), controller_parameters_.rate_gain_kp_.z());
      rate_gain_ki_private_ = Eigen::Vector3f(controller_parameters_.rate_gain_ki_.x(), controller_parameters_.rate_gain_ki_.y(), controller_parameters_.rate_gain_ki_.z());
      rate_gain_kd_private_ = Eigen::Vector3f(controller_parameters_.rate_gain_kd_.x(), controller_parameters_.rate_gain_kd_.y(), controller_parameters_.rate_gain_kd_.z());
      rate_integration_limit_ = Eigen::Vector3f(controller_parameters_.rate_integration_limit_.x(), controller_parameters_.rate_integration_limit_.y(), controller_parameters_.rate_integration_limit_.z());

      ROS_DEBUG("Attitude gains - Kp_p: %f, Ki_p: %f, Kp_q: %f, Ki_q: %f", attitude_gain_kp_private_.x(),
                 attitude_gain_ki_private_.x(), attitude_gain_kp_private_.y(), attitude_gain_ki_private_.y());
      ROS_DEBUG("Rate gains - Kp_phi: %f, Ki_phi: %f, Kp_theta: %f, Ki_theta: %f, Kp_psi: %f, Ki_psi: %f", rate_gain_kp_private_.x(),
                 rate_gain_ki_private_.x(), rate_gain_kp_private_.y(), rate_gain_ki_private_.y(), rate_gain_kp_private_.z(),
                 rate_gain_ki_private_.z());

      ROS_INFO("Attitude Integration limits - phi: %f, theta: %f", attitude_integration_limit_.x(), attitude_integration_limit_.y());

      ROS_INFO("Rate gains - Kp_phi: %f, Ki_phi: %f, Kd_phi: %f, Kp_theta: %f, Ki_theta: %f, Kd_theta: %f, Kp_psi: %f, Ki_psi: %f, Kd_psi: %f", rate_gain_kp_private_.x(),
                 rate_gain_ki_private_.x(), rate_gain_kd_private_.x(), rate_gain_kp_private_.y(), rate_gain_ki_private_.y(), rate_gain_kd_private_.y(), rate_gain_kp_private_.z(),
                 rate_gain_ki_private_.z(), rate_gain_kd_private_.z());

      ROS_INFO("Rate Integration limits - p: %f, q: %f, r: %f", rate_integration_limit_.x(), rate_integration_limit_.y(), rate_integration_limit_.z());

}

void CrazyflieOnboardController::RateController(double* roll_ctrl, double* pitch_ctrl, double* yaw_ctrl) {
    assert(roll_ctrl);
    assert(pitch_ctrl);
    assert(yaw_ctrl);

    double roll_rate, pitch_rate, yaw_rate;
    roll_rate = state_t_private_.angularVelocity.x * 180/M_PI;;
    pitch_rate = state_t_private_.angularVelocity.y * 180/M_PI;;
    yaw_rate = state_t_private_.angularVelocity.z * 180/M_PI;;

    // ROS_INFO("Yaw rate - %f, %f, %f", p, q, r);

    // p = true_ang_x;
    // q = true_ang_y;
    // r = true_ang_z;

    double yaw_rate_command;
    yaw_rate_command = control_t_private_.yawRate;

    // Update the p and q commands with a frequency rate of 250Hz. The rate controller works with a frequency rate of 500Hz
    // if(counter_){
    //    AttitudeController(&p_command_, &q_command_);
    //    counter_ = false;
    // }
    // else
    //   counter_ = true;
    double desired_roll_rate, desired_pitch_rate;
    AttitudeController(&desired_roll_rate, &desired_pitch_rate);

    pidSetDesired(&pidRollRate, (float) desired_roll_rate);
    pidSetDesired(&pidPitchRate, (float) desired_pitch_rate);
    pidSetDesired(&pidYawRate, (float) yaw_rate_command);

    *roll_ctrl = pidUpdate(&pidRollRate, (float) roll_rate, 1);
    *pitch_ctrl = pidUpdate(&pidPitchRate, (float) pitch_rate, 1);
    *yaw_ctrl = pidUpdate(&pidYawRate, (float) yaw_rate, 1);

    // double p_error, q_error, r_error;
    // p_error = p_command_ - p;
    // q_error = q_command_ - q;
    // r_error = r_command - r;

    // ROS_DEBUG("p_command: %f, q_command: %f", p_command_, q_command_);
    // ROS_INFO("p_command: %f, q_command: %f", p_command_, q_command_);

    // double delta_phi_kp, delta_theta_kp, delta_psi_kp;
    // delta_phi_kp = rate_gain_kp_private_.x() * p_error;
    // *delta_phi = delta_phi_kp;

    // delta_theta_kp = rate_gain_kp_private_.y() * q_error;
    // *delta_theta = delta_theta_kp;

    // delta_psi_kp = rate_gain_kp_private_.z() * r_error;
    // delta_psi_ki_ = delta_psi_ki_ + (rate_gain_ki_private_.z() * r_error * SAMPLING_TIME_RATE_CONTROLLER);
    // *delta_psi = delta_psi_kp + delta_psi_ki_;

    // ROS_INFO("phi: %f theta: %f psi: %f",*delta_phi, *delta_theta, *delta_psi);

    // ROS_INFO("p_error: %f, q_error: %f", p_error, q_error);
    // ROS_INFO("p = %f, q = %f", q, p);

}

// The attitude controller runs with a frequency rate of 250Hz
void CrazyflieOnboardController::AttitudeController(double* roll_cmd, double* pitch_cmd) {
    assert(roll_cmd);
    assert(pitch_cmd);

    
    // Quaternion2Euler(&roll, &pitch, &yaw);
    // roll = true_r;
    // pitch = true_p;
    // yaw = true_y;
    const float roll = (float) state_t_private_.attitude.roll * 180/M_PI;;
    const float pitch = (float) state_t_private_.attitude.pitch * 180/M_PI;;

    // ROS_INFO("Attitude: %f, %f", roll, pitch);
    // ROS_INFO("Target - %f", control_t_private_.roll);

    pidSetDesired(&pidRoll, (float) control_t_private_.roll);
    pidSetDesired(&pidPitch, (float) control_t_private_.pitch);

    *roll_cmd = pidUpdate(&pidRoll, roll, 1);
    *pitch_cmd = pidUpdate(&pidPitch, pitch, 1);

    // ROS_INFO("p_command: %f, q_command: %f", *p_command_internal, *q_command_internal);

    // ROS_INFO("Roll: %f Pitch: %f Yaw: %f", roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);

    // double theta_command, phi_command;
    // theta_command = control_t_private_.pitch;
    // phi_command = control_t_private_.roll;

    // double phi_error, theta_error;
    // phi_error = phi_command - roll;
    // theta_error = theta_command - pitch;

    // // Calculate 
    // double p_command_kp, q_command_kp;
    // p_command_kp = attitude_gain_kp_private_.x() * phi_error;
    // p_integ_ += phi_error * SAMPLING_TIME_ATTITUDE_CONTROLLER;
    // p_integ_ = limit_integ(p_integ_, attitude_integration_limit_.x());

    // p_command_ki_ = attitude_gain_ki_private_.x() * p_integ_;

    // float deriv = (phi_error - p_prev_error_) / SAMPLING_TIME_ATTITUDE_CONTROLLER;
    // p_prev_error_ = phi_error;
    // p_command_kd_ = attitude_gain_kd_private_.x()*deriv;

    // *p_command_internal = p_command_kp + p_command_ki_ + p_command_kd_;



    // ROS_INFO_ONCE("The complementary is running");

    // ROS_DEBUG("Phi_c: %f, Phi_e: %f, Theta_c: %f, Theta_e: %f", phi_command, phi_error, theta_command, theta_error);
    // ROS_DEBUG("p_command: %f, q_command: %f", *p_command_internal, *q_command_internal);
    // ROS_INFO("p_command: %f, q_command: %f", *p_command_internal, *q_command_internal);
    // ROS_INFO("Roll error: %f, Pitch error: %f", phi_error, theta_error );

}

void CrazyflieOnboardController::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    assert(roll);
    assert(pitch);
    assert(yaw);

    //The estimated quaternion values
    double x, y, z, w;
    x = state_t_private_.attitudeQuaternion.x;
    y = state_t_private_.attitudeQuaternion.y;
    z = state_t_private_.attitudeQuaternion.z;
    w = state_t_private_.attitudeQuaternion.w;

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);

    ROS_DEBUG("Roll: %f, Pitch: %f, Yaw: %f", *roll, *pitch, *yaw);
}

}
