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

#ifndef CRAYZFLIE_2_RPY_CONTROLLER_V2_H
#define CRAYZFLIE_2_RPY_CONTROLLER_V2_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/RollPitchYawrateThrustCrazyflie.h>

#include <string>

#include <ros/time.h>

#include "common.h"
#include "parameters.h"
#include "stabilizer_types.h"
#include "crazyflie_complementary_filter.h"
#include "crazyflie_onboard_controller_v2.h"
#include "sensfusion6.h"
#include "controller_parameters.h"

#include <time.h>

using namespace std;

namespace rotors_control {

    class CrazyflieRPYController{
        public:
            CrazyflieRPYController();
            ~CrazyflieRPYController();
            void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

            void SetOdometry(const EigenOdometry& odometry);
            // void UpdateState(const sensorData_t& sensors);
            void SetSensors(const sensorData_t& sensors);
            void UpdateState();
            void CallbackAttitudeEstimation();
            void SetLaunchFileParameters();

            void SetRollPitchYawrateThrust(const mav_msgs::EigenRollPitchYawrateThrustCrazyflie& roll_pitch_yawrate_thrust);

            void SetTrueRPYAngularVelocity();

            // void SetControllerGains();

            // Lunch file parameters
            std::string user_;
            double dataStoringTime_;
            bool dataStoring_active_;
            bool waypointFilter_active_;
            bool EKF_active_;

            double pwm1, pwm2, pwm3, pwm4;

            PositionControllerParameters controller_parameters_;
            ComplementaryFilterCrazyflie2 complementary_filter_crazyflie_;
            CrazyflieOnboardController crazyflie_onboard_controller_;

        private:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            bool controller_active_;
            bool state_estimator_active_;

            control_s control_t_;

            // Lists for data saving
            std::vector<string> listPropellersVelocity_;
            std::vector<string> listDroneAttiude_;
            std::vector<string> listPWM_;
            std::vector<string> listPWMComponents_;
            std::vector<string> listCommandAttiude_;
            std::vector<string> listRCommand_;
            std::vector<string> listOmegaCommand_;
            std::vector<string> listXeYe_;
            std::vector<string> listDeltaCommands_;
            std::vector<string> listPQCommands_;
            std::vector<string> listDronePosition_;

            // Callbacks
            ros::NodeHandle n_;
            ros::Timer timer_;
            void CallbackSaveData(const ros::TimerEvent& event);

            //Integrator initial conditions
            // double theta_command_ki_;
            // double phi_command_ki_;
            // double p_command_ki_;
            // double q_command_ki_;
            // double delta_psi_ki_;
            // double r_command_ki_;
            double delta_omega_ki_;

            //Controller gains
            // double hovering_gain_kp_, hovering_gain_ki_, hovering_gain_kd_;
            // double hovering_gain_kp_, hovering_gain_ki_, hovering_gain_kd_;

            // void SetSensorData();

            // mav_msgs::EigenTrajectoryPoint command_trajectory_;
            EigenOdometry odometry_;
            sensorData_t sensors_;
            state_t state_;

            void ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4);
            void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;

            mav_msgs::EigenRollPitchYawrateThrustCrazyflie roll_pitch_yawrate_thrust_;

    };

}
#endif // CRAYZFLIE_2_RPY_CONTROLLER_H
