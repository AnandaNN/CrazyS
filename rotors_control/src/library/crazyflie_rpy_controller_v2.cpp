/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
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

#include "rotors_control/crazyflie_rpy_controller_v2.h"
#include "rotors_control/transform_datatypes.h"
#include "rotors_control/Matrix3x3.h"
#include "rotors_control/Quaternion.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/sensfusion6.h"

#include <math.h>
#include <ros/ros.h>
#include <time.h>
#include <chrono>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>

#include <inttypes.h>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>


#define M_PI                                     3.14159265358979323846  /* pi [rad]*/
#define OMEGA_OFFSET                             0  /* OMEGA OFFSET [PWM]*/
#define ANGULAR_MOTOR_COEFFICIENT                0.04363 /* ANGULAR_MOTOR_COEFFICIENT */
#define MOTORS_INTERCEPT                         0.0/* MOTORS_INTERCEPT [rad/s]*/
#define MAX_PROPELLERS_ANGULAR_VELOCITY          5000 /* MAX PROPELLERS ANGULAR VELOCITY [rad/s]*/
#define MAX_PWM_VALUE                            80000
#define MAX_R_DESIDERED                          3.4907 /* MAX R DESIDERED VALUE [rad/s]*/
#define MAX_THETA_COMMAND                        0.5236 /* MAX THETA COMMMAND [rad]*/
#define MAX_PHI_COMMAND                          0.5236 /* MAX PHI COMMAND [rad]*/
#define MAX_POS_DELTA_OMEGA                      1289 /* MAX POSITIVE DELTA OMEGA [PWM]*/
#define MAX_NEG_DELTA_OMEGA                      -1718 /* MAX NEGATIVE DELTA OMEGA [PWM]*/
#define SAMPLING_TIME                            0.01 /* SAMPLING TIME [s] */

namespace rotors_control{

// CrazyflieRPYController::CrazyflieRPYController()
//     : controller_active_(false),
//     state_estimator_active_(true),
//     dataStoring_active_(false),
//     dataStoringTime_(0),
//     phi_command_ki_(0),
//     theta_command_ki_(0),
//     p_command_ki_(0),
//     q_command_ki_(0),
//     r_command_ki_(0),
//     delta_psi_ki_(0),
//     delta_omega_ki_(0),
//     hovering_gain_kd_(0){

CrazyflieRPYController::CrazyflieRPYController()
    : controller_active_(false),
    state_estimator_active_(true),
    dataStoring_active_(false),
    dataStoringTime_(0){

      // The control variables are initialized to zero
      control_t_.roll = 0;
      control_t_.pitch = 0;
      control_t_.yawRate = 0;
      control_t_.thrust = 0;

      state_.angularAcc.x = 0; // Angular Acceleration x
      state_.angularAcc.y = 0; // Angular Acceleration y
      state_.angularAcc.z = 0; // Angular Acceleration z

      state_.attitude.roll = 0; // Roll
      state_.attitude.pitch = 0; // Pitch
      state_.attitude.yaw = 0; // Yaw

      state_.position.x = 0; // Position.x
      state_.position.y = 0; // Position.y
      state_.position.z = 0; // Position.z

      state_.angularVelocity.x = 0; // Angular velocity x
      state_.angularVelocity.y = 0; // Angular velocity y
      state_.angularVelocity.z = 0; // Angular velocity z

      state_.linearVelocity.x = 0; //Linear velocity x
      state_.linearVelocity.y = 0; //Linear velocity y
      state_.linearVelocity.z = 0; //Linear velocity z

      state_.attitudeQuaternion.x = 0; // Quaternion x
      state_.attitudeQuaternion.y = 0; // Quaternion y
      state_.attitudeQuaternion.z = 0; // Quaternion z
      state_.attitudeQuaternion.w = 0; // Quaternion w

}

CrazyflieRPYController::~CrazyflieRPYController() {}

void CrazyflieRPYController::SetRollPitchYawrateThrust(const mav_msgs::EigenRollPitchYawrateThrustCrazyflie& roll_pitch_yawrate_thrust) {

    roll_pitch_yawrate_thrust_ = roll_pitch_yawrate_thrust;
    controller_active_ = true;

    control_t_.pitch = roll_pitch_yawrate_thrust_.pitch * 180/M_PI;
    control_t_.roll = roll_pitch_yawrate_thrust_.roll * 180/M_PI;
    control_t_.yawRate = roll_pitch_yawrate_thrust_.yaw_rate * 180/M_PI;
    control_t_.thrust = roll_pitch_yawrate_thrust_.thrust * 60000.0/8162.0;

    // ROS_INFO("GOT COMMAND: %f",control_t_.thrust);

}

//The callback saves data come from simulation into csv files
void CrazyflieRPYController::CallbackSaveData(const ros::TimerEvent& event){

      if(!dataStoring_active_){
         return;
      }

      ofstream filePropellersVelocity;
      ofstream fileDroneAttiude;
      ofstream filePWM;
      ofstream filePWMComponents;
      ofstream fileCommandAttiude;
      ofstream fileRCommand;
      ofstream fileOmegaCommand;
      ofstream fileXeYe;
      ofstream fileDeltaCommands;
      ofstream filePQCommands;
      ofstream fileDronePosition;

      ROS_INFO("CallbackSavaData function is working. Time: %f seconds, %f nanoseconds", odometry_.timeStampSec, odometry_.timeStampNsec);

      filePropellersVelocity.open("/home/" + user_ + "/PropellersVelocity.csv", std::ios_base::app);
      fileDroneAttiude.open("/home/" + user_ + "/DroneAttiude.csv", std::ios_base::app);
      filePWM.open("/home/" + user_ + "/PWM.csv", std::ios_base::app);
      filePWMComponents.open("/home/" + user_ + "/PWMComponents.csv", std::ios_base::app);
      fileCommandAttiude.open("/home/" + user_ + "/CommandAttitude.csv", std::ios_base::app);
      fileRCommand.open("/home/" + user_ + "/RCommand.csv", std::ios_base::app);
      fileOmegaCommand.open("/home/" + user_ + "/OmegaCommand.csv", std::ios_base::app);
      fileXeYe.open("/home/" + user_ + "/XeYe.csv", std::ios_base::app);
      fileDeltaCommands.open("/home/" + user_ + "/DeltaCommands.csv", std::ios_base::app);
      filePQCommands.open("/home/" + user_ + "/PQCommands.csv", std::ios_base::app);
      fileDronePosition.open("/home/" + user_ + "/DronePosition.csv", std::ios_base::app);

      // Saving control signals in a file
      for (unsigned n=0; n < listPropellersVelocity_.size(); ++n) {
          filePropellersVelocity << listPropellersVelocity_.at( n );
      }

      for (unsigned n=0; n < listDroneAttiude_.size(); ++n) {
          fileDroneAttiude << listDroneAttiude_.at( n );
      }

      for (unsigned n=0; n < listPWM_.size(); ++n) {
          filePWM << listPWM_.at( n );
      }

      for (unsigned n=0; n < listPWMComponents_.size(); ++n) {
          filePWMComponents << listPWMComponents_.at( n );
      }

      for (unsigned n=0; n < listCommandAttiude_.size(); ++n) {
          fileCommandAttiude << listCommandAttiude_.at( n );
      }

      for (unsigned n=0; n < listRCommand_.size(); ++n) {
          fileRCommand << listRCommand_.at( n );
      }

      for (unsigned n=0; n < listOmegaCommand_.size(); ++n) {
          fileOmegaCommand << listOmegaCommand_.at( n );
      }

      for (unsigned n=0; n < listXeYe_.size(); ++n) {
          fileXeYe << listXeYe_.at( n );
      }

      for (unsigned n=0; n < listDeltaCommands_.size(); ++n) {
          fileDeltaCommands << listDeltaCommands_.at( n );
      }

      for (unsigned n=0; n < listPQCommands_.size(); ++n) {
          filePQCommands << listPQCommands_.at( n );
      }

      for (unsigned n=0; n < listDronePosition_.size(); ++n) {
          fileDronePosition << listDronePosition_.at( n );
      }

      // Closing all opened files
      filePropellersVelocity.close();
      fileDroneAttiude.close();
      filePWM.close();
      filePWMComponents.close();
      fileCommandAttiude.close();
      fileRCommand.close();
      fileOmegaCommand.close();
      fileXeYe.close();
      fileDeltaCommands.close();
      filePQCommands.close();
      fileDronePosition.close();

      // To have a one shot storing
      dataStoring_active_ = false;

}

// Reading parameters come frame launch file
void CrazyflieRPYController::SetLaunchFileParameters(){

	// The boolean variable is used to inactive the logging if it is not useful
	if(dataStoring_active_){

		// Time after which the data storing function is turned on
		timer_ = n_.createTimer(ros::Duration(dataStoringTime_), &CrazyflieRPYController::CallbackSaveData, this, false, true);

		// Cleaning the string vector contents
    listPropellersVelocity_.clear();
    listDroneAttiude_.clear();
    listPWM_.clear();
    listPWMComponents_.clear();
    listCommandAttiude_.clear();
    listRCommand_.clear();
    listOmegaCommand_.clear();
    listXeYe_.clear();
    listDeltaCommands_.clear();
    listPQCommands_.clear();
    listDronePosition_.clear();

	}

}

void CrazyflieRPYController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
    assert(rotor_velocities);

    // This is to disable the controller if we do not receive controller data
    if(!controller_active_){
       *rotor_velocities = Eigen::Vector4d::Zero(rotor_velocities->rows());
        return;
    }

    double PWM_1, PWM_2, PWM_3, PWM_4;
    if(controller_active_) ControlMixer(&PWM_1, &PWM_2, &PWM_3, &PWM_4);

    double omega_1, omega_2, omega_3, omega_4;
    // omega_1 = ((PWM_1 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    // omega_2 = ((PWM_2 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    // omega_3 = ((PWM_3 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    // omega_4 = ((PWM_4 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);

    omega_1 = ((PWM_1 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    omega_2 = ((PWM_2 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    omega_3 = ((PWM_3 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    omega_4 = ((PWM_4 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);


    //The omega values are saturated considering physical constraints of the system
    if(!(omega_1 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_1 > 0))
        if(omega_1 > MAX_PROPELLERS_ANGULAR_VELOCITY){
           omega_1 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        //    ROS_INFO("MAX REACHED 1");
        }
        else
           omega_1 = 0;

    if(!(omega_2 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_2 > 0))
        if(omega_2 > MAX_PROPELLERS_ANGULAR_VELOCITY){
           omega_2 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        //    ROS_INFO("MAX REACHED 2");
        }
        else
           omega_2 = 0;

    if(!(omega_3 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_3 > 0))
        if(omega_3 > MAX_PROPELLERS_ANGULAR_VELOCITY){
           omega_3 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        //    ROS_INFO("MAX REACHED 3");
        }
        else
           omega_3 = 0;

    if(!(omega_4 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_4 > 0))
        if(omega_4 > MAX_PROPELLERS_ANGULAR_VELOCITY){
           omega_4 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        //    ROS_INFO("MAX REACHED 4");
        }
        else
           omega_4 = 0;

   if(dataStoring_active_){
     // Saving drone attitude in a file
     std::stringstream tempPropellersVelocity;
     tempPropellersVelocity << omega_1 << "," << omega_2 << "," << omega_3 << "," << omega_4 << ","
             << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

     listPropellersVelocity_.push_back(tempPropellersVelocity.str());
   }

    ROS_DEBUG("Omega_1: %f Omega_2: %f Omega_3: %f Omega_4: %f", omega_1, omega_2, omega_3, omega_4);
    // ROS_INFO("Omega_1: %f Omega_2: %f Omega_3: %f Omega_4: %f", omega_1, omega_2, omega_3, omega_4);
    // ROS_INFO("PWM_1: %f PWM_2: %f PWM_3: %f PWM_4: %f", PWM_1, PWM_2, PWM_3, PWM_4);
    *rotor_velocities = Eigen::Vector4d(omega_1, omega_2, omega_3, omega_4);
}

void CrazyflieRPYController::ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4) {
    assert(PWM_1);
    assert(PWM_2);
    assert(PWM_3);
    assert(PWM_4);

    // Control signals are sent to the on board control architecture if the state estimator is active
    double roll_ctrl, pitch_ctrl, yaw_ctrl;
    
    // if(state_estimator_active_){
       
    // }

    // ROS_INFO("Thrust = %f", control_t_.thrust);

    if( control_t_.thrust > 1000 ) {
        crazyflie_onboard_controller_.SetDroneState(state_);
        crazyflie_onboard_controller_.SetControlSignals(control_t_);
        crazyflie_onboard_controller_.RateController(&roll_ctrl, &pitch_ctrl, &yaw_ctrl);
        
        ROS_INFO("thrust: %f,  dphi: %f,  dtheta: %f,  dpsi: %f", control_t_.thrust, roll_ctrl, pitch_ctrl, yaw_ctrl);

        // else
        //    RateController(&delta_phi, &delta_theta, &delta_psi);

        *PWM_1 = control_t_.thrust - (pitch_ctrl/2) - (roll_ctrl/2) - yaw_ctrl;
        *PWM_2 = control_t_.thrust + (pitch_ctrl/2) - (roll_ctrl/2) + yaw_ctrl;
        *PWM_3 = control_t_.thrust + (pitch_ctrl/2) + (roll_ctrl/2) - yaw_ctrl;
        *PWM_4 = control_t_.thrust - (pitch_ctrl/2) + (roll_ctrl/2) + yaw_ctrl;
        // ROS_INFO("PWM1: %f, PWM2: %f, PWM3: %f, PWM4: %f", *PWM_1, *PWM_2, *PWM_3, *PWM_4);
        // *PWM_1 = control_t_.thrust;
        // *PWM_2 = control_t_.thrust;
        // *PWM_3 = control_t_.thrust;
        // *PWM_4 = control_t_.thrust;
    }
    else {
        *PWM_1 = 0;
        *PWM_2 = 0;
        *PWM_3 = 0;
        *PWM_4 = 0;
        crazyflie_onboard_controller_.ResetPIDControllers();
    }

    if( *PWM_1 > MAX_PWM_VALUE ) *PWM_1 = MAX_PWM_VALUE;
    else if ( *PWM_1 < 0 ) *PWM_1 = 0;
    if( *PWM_2 > MAX_PWM_VALUE ) *PWM_2 = MAX_PWM_VALUE;
    else if ( *PWM_2 < 0 ) *PWM_2 = 0;
    if( *PWM_3 > MAX_PWM_VALUE ) *PWM_3 = MAX_PWM_VALUE;
    else if ( *PWM_3 < 0 ) *PWM_3 = 0;
    if( *PWM_4 > MAX_PWM_VALUE ) *PWM_4 = MAX_PWM_VALUE;
    else if ( *PWM_4 < 0 ) *PWM_4 = 0;

    pwm1 = *PWM_1;
    pwm2 = *PWM_2;
    pwm3 = *PWM_3;
    pwm4 = *PWM_4;

    // *PWM_1 = control_t_.thrust - (delta_theta/1) - (delta_phi/1) - delta_psi;
    // *PWM_2 = control_t_.thrust + (delta_theta/1) - (delta_phi/1) + delta_psi;
    // *PWM_3 = control_t_.thrust + (delta_theta/1) + (delta_phi/1) - delta_psi;
    // *PWM_4 = control_t_.thrust - (delta_theta/1) + (delta_phi/1) + delta_psi;

    if(dataStoring_active_){
      // Saving drone attitude in a file
      std::stringstream tempPWM;
      tempPWM << *PWM_1 << "," << *PWM_2 << "," << *PWM_3 << "," << *PWM_4 << ","
              << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listPWM_.push_back(tempPWM.str());

      // Saving drone attitude in a file
      std::stringstream tempPWMComponents;
      tempPWMComponents << control_t_.thrust << "," << pitch_ctrl << "," << roll_ctrl << "," << yaw_ctrl << ","
              << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listPWMComponents_.push_back(tempPWMComponents.str());
    }

    // ROS_INFO("ControlMixer: %f", control_t_.thrust);
    // ROS_DEBUG("Omega: %f, Delta_theta: %f, Delta_phi: %f, delta_psi: %f", control_t_.thrust, delta_theta, delta_phi, delta_psi);
    ROS_DEBUG("PWM1: %f, PWM2: %f, PWM3: %f, PWM4: %f", *PWM_1, *PWM_2, *PWM_3, *PWM_4);
    // ROS_INFO("PWM1: %f, PWM2: %f, PWM3: %f, PWM4: %f", *PWM_1, *PWM_2, *PWM_3, *PWM_4);
}

void CrazyflieRPYController::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    assert(roll);
    assert(pitch);
    assert(yaw);

    // The estimated quaternion values
    double x, y, z, w;
    x = state_.attitudeQuaternion.x;
    y = state_.attitudeQuaternion.y;
    z = state_.attitudeQuaternion.z;
    w = state_.attitudeQuaternion.w;

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);

    ROS_DEBUG("Roll: %f, Pitch: %f, Yaw: %f", *roll, *pitch, *yaw);

}

/* FROM HERE THE FUNCTIONS EMPLOYED WHEN THE STATE ESTIMATOR IS ABLED ARE REPORTED */

// Such function is invoked by the position controller node when the state estimator is considered in the loop
void CrazyflieRPYController::SetOdometry(const EigenOdometry& odometry) {

    odometry_ = odometry;
}


// The aircraft attitude is computed by the complementary filter with a frequency rate of 250Hz
void CrazyflieRPYController::CallbackAttitudeEstimation() {

    // Angular velocities updating
    // complementary_filter_crazyflie_.EstimateAttitude(&state_, &sensors_);

    ROS_DEBUG("Attitude Callback");

}

void CrazyflieRPYController::SetTrueRPYAngularVelocity()
{
    // crazyflie_onboard_controller_.true_gyro = Eigen::Vector3f( odometry_.angular_velocity.x(), odometry_.angular_velocity.y(), odometry_.angular_velocity.z() );

    //The estimated quaternion values
    double x, y, z, w;
    double roll, pitch, yaw;
    x = odometry_.orientation.x();
    y = odometry_.orientation.y();
    z = odometry_.orientation.z();
    w = odometry_.orientation.w();
    
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    crazyflie_onboard_controller_.true_r = roll;
    crazyflie_onboard_controller_.true_p = pitch;
    crazyflie_onboard_controller_.true_y = yaw;

    crazyflie_onboard_controller_.true_ang_x = odometry_.angular_velocity.x();
    crazyflie_onboard_controller_.true_ang_y = odometry_.angular_velocity.y();
    crazyflie_onboard_controller_.true_ang_z = odometry_.angular_velocity.z();

    state_.attitude.roll = roll;
    state_.attitude.pitch = pitch;
    state_.attitude.yaw = yaw;

    // ROS_INFO("%f %f %f", roll, pitch, yaw);

    // crazyflie_onboard_controller_.true_rpy = Eigen::Vector3f(*roll, *pitch, *yaw);
}

void CrazyflieRPYController::SetSensors(const sensorData_t& sensors) {
    sensors_ = sensors;
}

// The aircraft angular velocities are update with a frequency of 500Hz
// void CrazyflieRPYController::UpdateState(const sensorData_t& sensors) {
void CrazyflieRPYController::UpdateState() {

    // The functions runs at 500Hz, the same frequency with which the IMU topic publishes new values (with a frequency of 500Hz)
    // sensors_ = sensors;
    // complementary_filter_crazyflie_.EstimateRate(&state_, &sensors_);

    // if(!state_estimator_active_)
    //     state_estimator_active_= true;

    // Only the position sensor is ideal, any virtual sensor or systems is available to get these data
    // Every 0.002 seconds the odometry message values are copied in the state_ structure, but they change only 0.01 seconds
    state_.position.x = odometry_.position[0];
    state_.position.y = odometry_.position[1];
    state_.position.z = odometry_.position[2];

    state_.linearVelocity.x = odometry_.velocity[0];
    state_.linearVelocity.y = odometry_.velocity[1];
    state_.linearVelocity.z = odometry_.velocity[2];

    state_.attitudeQuaternion.x = odometry_.orientation.x();
    state_.attitudeQuaternion.y = odometry_.orientation.y();
    state_.attitudeQuaternion.z = odometry_.orientation.z();
    state_.attitudeQuaternion.w = odometry_.orientation.w();

    state_.angularVelocity.x = odometry_.angular_velocity[0];
    state_.angularVelocity.y = odometry_.angular_velocity[1];
    state_.angularVelocity.z = odometry_.angular_velocity[2];

    SetTrueRPYAngularVelocity();

}


}
