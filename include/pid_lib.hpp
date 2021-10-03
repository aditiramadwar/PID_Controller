#ifndef INCLUDE_PID_LIB_HPP_
#define INCLUDE_PID_LIB_HPP_

/**
 * Copyright (C) 2021 by Aditi Ramadwar
 * @file    pid_lib.hpp
 * @author  Part 1 : Aditi Ramadwar (Driver) , Yash Kulkarni (Navigator)
 * @date    9/30/2021
 * @version 1.0
 *
 * @brief   ControllerPID class.
 *
 * @section DESCRIPTION
 * This hpp file defines the class and methods for a PID Controller.
 */

// Header file
#include<iostream>

/**
 * @brief  PID Controller Class, initialization of the gain values
 *
 * @param1 k_p This is the proportional gain constant, user input
 * @param2 k_d This is the derivative gain constant, user input
 * @param3 k_i This is the integral gain constant, user input
 * @param4 sampling_time 
 */
class ControllerPID {
 public:
       double computeVelocity(double set_point, double current_velocity);
       double returnSamplingTime(void);
       ControllerPID(double k_p_, double k_i_, double k_d_) {
        // Initialize K_p, K_i, k_d
        k_p = k_p_;
        k_i = k_i_;
        k_d_ = k_d_;
        sampling_time = 1.0;
        cur_error = 0;
        prev_error = 0;
        new_velocity = 0;
        total_D_error = 0;
        total_I_error = 0;
    }
 
 private:
        double k_p = 1.2;
        double k_d = 0.2;
        double k_i = 0.4;
        double sampling_time = 1.0;
        double prev_error;
        double cur_error;
        double new_velocity;
        double total_D_error;
        double total_I_error;
};

/** 
* @def computeVelocity(double set_point, double current_velocity)
* 
* @brief
* This method computes the new velocity using current and reference velocities as inputs
* This is stub implementation of the method computeVelocity which returns a constant value.
* We need to add the PID computations inside of this method in order for the test cases to pass.
* 
* @param1 set_point         This is the target/desired velocity which needs to be achieved.
*                           This parameter will be taken from the user as input.
* 
* @param2 current_velocity  This is the current/actual velocity.
* 
* @return new_velocity
*/
double ControllerPID::computeVelocity(double set_point,
    double current_velocity) {
    cur_error = (set_point - current_velocity);
    total_I_error = total_I_error + cur_error*sampling_time;
    total_D_error = (cur_error - prev_error)/sampling_time;
    new_velocity = k_p*cur_error +  k_i*total_I_error + k_d*total_D_error;
    return new_velocity;
}

/** 
* @def returnSamplingTime(void)
* 
* @brief 
* This method returns the value of sampling_time that is used in the computation
* of new_velocity
* 
* @return sampling_time
*/
double ControllerPID::returnSamplingTime(void) {
       
       	return sampling_time;
}

#endif  // INCLUDE_PID_LIB_HPP_
