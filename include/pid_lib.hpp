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
#pragma once
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
    // Create method for computing velocity throigh PID
    double computeVelocity(double set_point, double current_velocity);
    // Return the sampling time value
    double returnSamplingTime(void);
    // Construct a new PID object
    ControllerPID(double k_p_, double k_i_, double k_d_);
 private:
        // Initializing the gain values, sampling time and PID errors
        double k_p;
        double k_d;
        double k_i;
        double sampling_time;
        double prev_error; // Global variable for initial 0
        double total_I_error; // Global variable for initial 0
};
#endif  // INCLUDE_PID_LIB_HPP_
