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

#ifndef CRAZYFLIE_2_ONBOARD_CONTROLLER_V2_H
#define CRAZYFLIE_2_ONBOARD_CONTROLLER_V2_H

#include "stabilizer_types.h"
#include "controller_parameters.h"
#include "rotors_control/pid.h"

namespace rotors_control {

    class CrazyflieOnboardController{
        public:
            
            CrazyflieOnboardController();
            ~CrazyflieOnboardController();

            void SetControlSignals(const control_s& control_t);
            void SetDroneState(const state_s& state_t);
            void SetControllerGains(PositionControllerParameters& controller_parameters_);
            void RateController(double* roll_ctrl, double* pitch_ctrl, double* yaw_ctrl);
 
            void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;

            void ResetPIDControllers();

            double true_r = 0;
            double true_p = 0;
            double true_y = 0;

            double true_ang_x = 0;
            double true_ang_y = 0;
            double true_ang_z = 0;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:

            control_s control_t_private_;
            state_s state_t_private_;
 
            // counter for critical section. It allows to synchronize the attitude and rate parts of
            // the crazyflie on board controller.
            bool counter_;          

            //Integrator intial condition
            double delta_psi_ki_;
            
            double p_integ_, q_integ_;
            double p_prev_error_, q_prev_error_;
            double p_command_, q_command_;
            double p_command_ki_, q_command_ki_;
            double p_command_kd_, q_command_kd_;

            PidObject pidRollRate;
            PidObject pidPitchRate;
            PidObject pidYawRate;
            PidObject pidRoll;
            PidObject pidPitch;

            Eigen::Vector2f attitude_gain_kp_private_, attitude_gain_ki_private_, attitude_gain_kd_private_, attitude_integration_limit_;
            Eigen::Vector3f rate_gain_kp_private_, rate_gain_ki_private_, rate_gain_kd_private_, rate_integration_limit_;

            void AttitudeController(double* roll_cmd, double* pitch_cmd);

     };

}

#endif // CRAZYFLIE_2_ONBOARD_CONTROLLER_H
