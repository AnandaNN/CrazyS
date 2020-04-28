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

#ifndef CRAYZFLIE_2_RPY_CONTROLLER_NODE_V2_H
#define CRAYZFLIE_2_RPY_CONTROLLER_NODE_V2_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/RollPitchYawrateThrustCrazyflie.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <ros/time.h>


#include "rotors_control/common.h"
#include "rotors_control/crazyflie_rpy_controller_v2.h"
// #include "rotors_control/crazyflie_complementary_filter.h"


namespace rotors_control {

    class CrazyflieRPYControllerNode{
        public:
            CrazyflieRPYControllerNode();
            ~CrazyflieRPYControllerNode();
             
            void InitializeParams();
            void Publish();

        private:

            bool waypointHasBeenPublished_ = false;
            bool enable_state_estimator_;

            CrazyflieRPYController rpy_controller_;
            sensorData_t sensors_;
            ros::Time imu_msg_head_stamp_;

            std::string namespace_;

            ros::NodeHandle n_;
            ros::Timer timer_Attitude_;
            ros::Timer timer_highLevelControl;
            ros::Timer timer_IMUUpdate;

            //Callback functions to compute the errors among axis and angles
            void CallbackAttitudeEstimation(const ros::TimerEvent& event);
            // void CallbackHightLevelControl(const ros::TimerEvent& event);
            void CallbackControlLoop(const ros::TimerEvent& event);
 
            void PublishStateAttitude();
            void PublishPWMValues();

            //subscribers
            ros::Subscriber odometry_sub_;
            ros::Subscriber imu_sub_;
            ros::Subscriber cmd_roll_pitch_yawrate_thrust_sub_;

            //publisher
            ros::Publisher motor_velocity_reference_pub_;
            ros::Publisher state_attitude_pub_;
            ros::Publisher pwm_value_pub_;

            mav_msgs::EigenTrajectoryPointDeque commands_;
            std::deque<ros::Duration> command_waiting_times_;
            ros::Timer command_timer_;

            void RollPitchYawrateThrustCallback(const mav_msgs::RollPitchYawrateThrustCrazyflieConstPtr& roll_pitch_yawrate_thrust_crazyflie_reference_msg);

            // void MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);

            void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

            void IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg);

    };
}

#endif // CRAYZFLIE_2_RPY_CONTROLLER_NODE_H
