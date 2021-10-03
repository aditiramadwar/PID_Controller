/**
* Copyright (C) 2021 by Aditi Ramadwar
* @file    test.cpp
* @author  Part 1 : Aditi Ramadwar (Driver) , Yash Kulkarni (Navigator)
* @date    9/30/2021
* @version 1.0
*
* @brief   Test cases
*
* @section DESCRIPTION
*
* This is a test for ControllerPID class and its methods.
* 
* Test cases:
* 
* Test case 1 tests the compute method by returning a double value which
* is pre-calculated. Ideally, after the changes are done in the computeVelocity
* method, this test should pass.
* 
* Test case 2 tests the correct initialization of sampling_time. This test
* passes even before doing any changes.
*/

// Header files
#include <gtest/gtest.h>
#include "pid_lib.hpp"

// Creating object pid of class ControllerPID


/**
 * @brief: Tests the compute method of the class by returning a double value,
 * should pass after correct changes in the computeVelocity method.
 * set_point = 8.0, current_velocity = 4.0, 
 * k_p, k_i, k_d, sampling_time = [1.2, 0.4, 0.2, 1]
 * expected return value = 7.2
 *
 * */
TEST(ComputeTest1, should_pass) {
  ControllerPID pid(1.2, 0.4, 0.2);
  ASSERT_EQ(6.4, pid.computeVelocity(8.0, 4.0));
}

/**
 * @brief: Tests the correct initialization of sampling_time
 * should pass
 * expected return value = 1.0
 */
TEST(ComputeTest2, should_pass) {
  ControllerPID pid(1.2, 0.2, 0.4);
  ASSERT_EQ(1.0, pid.returnSamplingTime());
}
