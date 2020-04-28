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

#include "crazyflie_rpy_controller_node_v2.h"

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>

#include "std_msgs/Float64MultiArray.h"

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/crazyflie_complementary_filter.h"

#define ATTITUDE_EST_UPDATE_DT 0.004  /* ATTITUDE UPDATE RATE [s] - 500Hz */
#define RATE_UPDATE_DT 0.002      /* RATE UPDATE RATE [s] - 250Hz */

namespace rotors_control {

CrazyflieRPYControllerNode::CrazyflieRPYControllerNode() {

    ROS_INFO_ONCE("Started position controller");

    InitializeParams();

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    // Read and set the enbale state estimator flag 
    if(pnh_node.getParam("enable_state_estimator", enable_state_estimator_)){
      ROS_INFO("Got param 'enable_state_estimator': %d", enable_state_estimator_);
    }

    // Used to set the true RPY to be used without state estimator
    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &CrazyflieRPYControllerNode::OdometryCallback, this);

    // Publish motor velocities
    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    // State estimation attitude publisher if enabled
    // if( enable_state_estimator_ )
    state_attitude_pub_ = nh.advertise<geometry_msgs::Vector3>("state_orientation_rpy", 1);

    pwm_value_pub_ = nh.advertise<std_msgs::Float64MultiArray>("pwm_value_control", 1);

    // Get RPY thurst commands from controller
    cmd_roll_pitch_yawrate_thrust_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1,
                                     &CrazyflieRPYControllerNode::RollPitchYawrateThrustCallback, this);


    // Read IMU data on callback
    imu_sub_ = nh.subscribe(mav_msgs::default_topics::IMU, 1, &CrazyflieRPYControllerNode::IMUCallback, this);

    // Set the state state estimator timer if enabled
    if( enable_state_estimator_ )
        timer_Attitude_ = n_.createTimer(ros::Duration(ATTITUDE_EST_UPDATE_DT), &CrazyflieRPYControllerNode::CallbackAttitudeEstimation, this, false, true);

    // The control loop timer at 500Hz
    timer_IMUUpdate = n_.createTimer(ros::Duration(RATE_UPDATE_DT), &CrazyflieRPYControllerNode::CallbackControlLoop, this, false, true);

}

CrazyflieRPYControllerNode::~CrazyflieRPYControllerNode(){}

void CrazyflieRPYControllerNode::RollPitchYawrateThrustCallback(const mav_msgs::RollPitchYawrateThrustCrazyflieConstPtr& roll_pitch_yawrate_thrust_crazyflie_reference_msg) {

  ROS_INFO_ONCE("GOT FIRST RPY-THRUST-COMMAND");

  waypointHasBeenPublished_ = true;

  mav_msgs::EigenRollPitchYawrateThrustCrazyflie roll_pitch_yawrate_thrust;
  mav_msgs::eigenRollPitchYawrateThrustCrazyflieFromMsg(*roll_pitch_yawrate_thrust_crazyflie_reference_msg, &roll_pitch_yawrate_thrust);
  rpy_controller_.SetRollPitchYawrateThrust(roll_pitch_yawrate_thrust);


}

void CrazyflieRPYControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Parameters reading from rosparam.
  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  rpy_controller_.controller_parameters_.attitude_gain_kp_.x(),
                  &rpy_controller_.controller_parameters_.attitude_gain_kp_.x());
  GetRosParameter(pnh, "attitude_gain_kp/theta",
                  rpy_controller_.controller_parameters_.attitude_gain_kp_.y(),
                  &rpy_controller_.controller_parameters_.attitude_gain_kp_.y());
  GetRosParameter(pnh, "attitude_gain_ki/phi",
                  rpy_controller_.controller_parameters_.attitude_gain_ki_.x(),
                  &rpy_controller_.controller_parameters_.attitude_gain_ki_.x());
  GetRosParameter(pnh, "attitude_gain_ki/theta",
                  rpy_controller_.controller_parameters_.attitude_gain_ki_.y(),
                  &rpy_controller_.controller_parameters_.attitude_gain_ki_.y());
  GetRosParameter(pnh, "attitude_gain_kd/phi",
                  rpy_controller_.controller_parameters_.attitude_gain_kd_.x(),
                  &rpy_controller_.controller_parameters_.attitude_gain_kd_.x());
  GetRosParameter(pnh, "attitude_gain_kd/theta",
                  rpy_controller_.controller_parameters_.attitude_gain_kd_.y(),
                  &rpy_controller_.controller_parameters_.attitude_gain_kd_.y());
  GetRosParameter(pnh, "attitude_integration_limit/phi",
                  rpy_controller_.controller_parameters_.attitude_integration_limit_.x(),
                  &rpy_controller_.controller_parameters_.attitude_integration_limit_.x());
  GetRosParameter(pnh, "attitude_integration_limit/theta",
                  rpy_controller_.controller_parameters_.attitude_integration_limit_.y(),
                  &rpy_controller_.controller_parameters_.attitude_integration_limit_.y());

  GetRosParameter(pnh, "rate_gain_kp/p",
                  rpy_controller_.controller_parameters_.rate_gain_kp_.x(),
                  &rpy_controller_.controller_parameters_.rate_gain_kp_.x());
  GetRosParameter(pnh, "rate_gain_kp/q",
                  rpy_controller_.controller_parameters_.rate_gain_kp_.y(),
                  &rpy_controller_.controller_parameters_.rate_gain_kp_.y());
  GetRosParameter(pnh, "rate_gain_kp/r",
                  rpy_controller_.controller_parameters_.rate_gain_kp_.z(),
                  &rpy_controller_.controller_parameters_.rate_gain_kp_.z());
  GetRosParameter(pnh, "rate_gain_ki/p",
                  rpy_controller_.controller_parameters_.rate_gain_ki_.x(),
                  &rpy_controller_.controller_parameters_.rate_gain_ki_.x());
  GetRosParameter(pnh, "rate_gain_ki/q",
                  rpy_controller_.controller_parameters_.rate_gain_ki_.y(),
                  &rpy_controller_.controller_parameters_.rate_gain_ki_.y());
  GetRosParameter(pnh, "rate_gain_ki/r",
                  rpy_controller_.controller_parameters_.rate_gain_ki_.z(),
                  &rpy_controller_.controller_parameters_.rate_gain_ki_.z());
  GetRosParameter(pnh, "rate_gain_kd/p",
                  rpy_controller_.controller_parameters_.rate_gain_kd_.x(),
                  &rpy_controller_.controller_parameters_.rate_gain_kd_.x());
  GetRosParameter(pnh, "rate_gain_kd/q",
                  rpy_controller_.controller_parameters_.rate_gain_kd_.y(),
                  &rpy_controller_.controller_parameters_.rate_gain_kd_.y());
  GetRosParameter(pnh, "rate_gain_kd/r",
                  rpy_controller_.controller_parameters_.rate_gain_kd_.z(),
                  &rpy_controller_.controller_parameters_.rate_gain_kd_.z());
  GetRosParameter(pnh, "rate_integration_limit/p",
                  rpy_controller_.controller_parameters_.rate_integration_limit_.x(),
                  &rpy_controller_.controller_parameters_.rate_integration_limit_.x());
  GetRosParameter(pnh, "rate_integration_limit/q",
                  rpy_controller_.controller_parameters_.rate_integration_limit_.y(),
                  &rpy_controller_.controller_parameters_.rate_integration_limit_.y());
  GetRosParameter(pnh, "rate_integration_limit/r",
                  rpy_controller_.controller_parameters_.rate_integration_limit_.z(),
                  &rpy_controller_.controller_parameters_.rate_integration_limit_.z());


  rpy_controller_.crazyflie_onboard_controller_.SetControllerGains(rpy_controller_.controller_parameters_);

  //Reading the parameters come from the launch file
  bool dataStoringActive;
  double dataStoringTime;
  std::string user;

  if (pnh.getParam("user_account", user)){
	  ROS_INFO("Got param 'user_account': %s", user.c_str());
	  rpy_controller_.user_ = user;
  }
  else
      ROS_ERROR("Failed to get param 'user'");

  if (pnh.getParam("csvFilesStoring", dataStoringActive)){
	  ROS_INFO("Got param 'csvFilesStoring': %d", dataStoringActive);
	  rpy_controller_.dataStoring_active_ = dataStoringActive;
  }
  else
      ROS_ERROR("Failed to get param 'csvFilesStoring'");

  if (pnh.getParam("csvFilesStoringTime", dataStoringTime)){
	  ROS_INFO("Got param 'csvFilesStoringTime': %f", dataStoringTime);
	  rpy_controller_.dataStoringTime_ = dataStoringTime;
  }
  else
      ROS_ERROR("Failed to get param 'csvFilesStoringTime'");

  rpy_controller_.SetLaunchFileParameters();

}

void CrazyflieRPYControllerNode::Publish(){
}

void CrazyflieRPYControllerNode::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

    ROS_INFO_ONCE("PositionController got first imu message.");

    // Angular velocities data
    sensors_.gyro.x = imu_msg->angular_velocity.x;
    sensors_.gyro.y = imu_msg->angular_velocity.y;
    sensors_.gyro.z = imu_msg->angular_velocity.z;

    // Linear acceleration data
    sensors_.acc.x = imu_msg->linear_acceleration.x;
    sensors_.acc.y = imu_msg->linear_acceleration.y;
    sensors_.acc.z = imu_msg->linear_acceleration.z;

    rpy_controller_.SetSensors(sensors_);

    imu_msg_head_stamp_ = imu_msg->header.stamp;

}

void CrazyflieRPYControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("PositionController got first odometry message.");

    //This functions allows us to put the odometry message into the odometry variable--> _position,
    //_orientation,_velocit_body,_angular_velocity
    EigenOdometry odometry;
    eigenOdometryFromMsg(odometry_msg, &odometry);
    rpy_controller_.SetOdometry(odometry); // Copies odometry data to local struct
    
    // rpy_controller_.SetTrueRPYAngularVelocity(); // Sets roll pitch and yaw from odometry data

}

// The attitude is estimated only if the waypoint has been published
// Only if enable state estimator is active
void CrazyflieRPYControllerNode::CallbackAttitudeEstimation(const ros::TimerEvent& event){

    ROS_INFO_ONCE("First Attitude Estimate Callback.");

    if (waypointHasBeenPublished_)
    {
        rpy_controller_.CallbackAttitudeEstimation();

    }

}

void CrazyflieRPYControllerNode::PublishStateAttitude() {
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;

    rpy_controller_.crazyflie_onboard_controller_.Quaternion2Euler(&roll, &pitch, &yaw);

    double yaw_degrees = yaw;// * 180.0 / M_PI; // conversion to degrees
    // if( yaw_degrees < 0 ) yaw_degrees += 360.0; // convert negative to positive angles

    double roll_degrees = roll;// * 180.0 / M_PI; // conversion to degrees
    // if( roll_degrees < 0 ) roll_degrees += 360.0; // convert negative to positive angles

    double pitch_degrees = pitch;// * 180.0 / M_PI; // conversion to degrees
    // if( pitch_degrees < 0 ) pitch_degrees += 360.0; // convert negative to positive angles

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll_degrees;
    rpy.y = pitch_degrees;
    rpy.z = yaw_degrees;

    // this Vector is then published:
    state_attitude_pub_.publish(rpy);
    ROS_DEBUG("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);
}

void CrazyflieRPYControllerNode::PublishPWMValues() {
    // the tf::Quaternion has a method to acess roll pitch and yaw

    std_msgs::Float64MultiArray pwm;

    pwm.data = {rpy_controller_.pwm1, rpy_controller_.pwm2, rpy_controller_.pwm3, rpy_controller_.pwm4};

    // this Vector is then published:
    pwm_value_pub_.publish(pwm);
    
}


// IMU messages are sent to the controller with a frequency of 500Hz. In other words, with a sampling time of 0.002 seconds
void CrazyflieRPYControllerNode::CallbackControlLoop(const ros::TimerEvent& event){

    rpy_controller_.UpdateState();

    PublishStateAttitude();

    ROS_INFO_ONCE("IMU Message sent to position controller");

    if (waypointHasBeenPublished_){

	    Eigen::Vector4d ref_rotor_velocities;
	    rpy_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

	    // A new mav message, actuator_msg, is used to send to Gazebo the propellers angular velocities.
	    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

	    // The clear method makes sure the actuator_msg is empty (there are no previous values of the propellers angular velocities).
	    actuator_msg->angular_velocities.clear();
	    // for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
	    for (int i = 0; i < ref_rotor_velocities.size(); i++)
	       actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
	    actuator_msg->header.stamp = imu_msg_head_stamp_;

	    motor_velocity_reference_pub_.publish(actuator_msg);

    }

    PublishPWMValues();

}


}

int main(int argc, char** argv){
    ros::init(argc, argv, "crazyflie_rpy_controller_node_with_stateEstimator");

    ros::NodeHandle nh2;

    rotors_control::CrazyflieRPYControllerNode crazyflie_rpy_controller_node;

    ros::spin();

    return 0;
}
