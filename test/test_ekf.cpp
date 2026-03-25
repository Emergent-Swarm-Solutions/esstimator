/*
 * Copyright (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <limits>
#include <vector>
#include <memory>

#include <esstimator/filter_base.hpp>
#include "esstimator/ekf.hpp"
#include "esstimator/ros_filter.hpp"
#include "esstimator/ros_filter_types.hpp"

using robot_localization::Ekf;
using robot_localization::RosEkf;
using robot_localization::STATE_SIZE;

TEST(EkfTest, Measurements) {
  // node handle is created as per ros2
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  std::shared_ptr<robot_localization::RosEkf> filter =
    std::make_shared<robot_localization::RosEkf>(options);
  filter->initialize();

  // create the instance of the class and pass parameters
  Eigen::MatrixXd initialCovar(15, 15);

  initialCovar.setIdentity();
  initialCovar *= 0.5;

  filter->getFilter().setEstimateErrorCovariance(initialCovar);

  Eigen::VectorXd measurement(STATE_SIZE);
  measurement.setIdentity();

  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurement[i] = i * 0.01 * STATE_SIZE;
  }
  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  measurementCovariance.setIdentity();
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurementCovariance(i, i) = 1e-9;
  }
  std::vector<bool> updateVector(STATE_SIZE, true);

  // Ensure that measurements are being placed in the queue correctly
  rclcpp::Time time1(1000);
  filter->robot_localization::RosEkf::enqueueMeasurement(
    "odom0", measurement, measurementCovariance, updateVector,
    std::numeric_limits<double>::max(), time1);

  filter->robot_localization::RosEkf::integrateMeasurements(rclcpp::Time(1001));

  EXPECT_EQ(filter->getFilter().getState(), measurement);
  EXPECT_EQ(
    filter->getFilter().getEstimateErrorCovariance(),
    measurementCovariance);

  filter->getFilter().setEstimateErrorCovariance(initialCovar);

  // Now fuse another measurement and check the output.
  // We know what the filter's state should be when
  // this is complete, so we'll check the difference and
  // make sure it's suitably small.
  Eigen::VectorXd measurement2 = measurement;

  measurement2 *= 2.0;

  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurementCovariance(i, i) = 1e-9;
  }

  rclcpp::Time time2(1002);

  filter->robot_localization::RosEkf::enqueueMeasurement(
    "odom0", measurement2, measurementCovariance, updateVector,
    std::numeric_limits<double>::max(), time2);

  filter->robot_localization::RosEkf::integrateMeasurements(rclcpp::Time(1003));

  measurement = measurement2.eval() - filter->getFilter().getState();
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    EXPECT_LT(::fabs(measurement[i]), 0.001);
  }
}

TEST(EkfTest, MahalanobisResultCallback) {
  Ekf filter;

  bool callback_called = false;
  bool callback_passed = true;
  std::string callback_stream;
  double callback_distance = 0.0;
  double callback_threshold = 0.0;
  rclcpp::Time callback_time(0, 0, RCL_ROS_TIME);

  filter.setMahalanobisResultCallback(
    [&](const std::string & stream_name,
      const double mahalanobis_distance,
      const double mahalanobis_threshold,
      const bool passed,
      const rclcpp::Time & measurement_time)
    {
      callback_called = true;
      callback_passed = passed;
      callback_stream = stream_name;
      callback_distance = mahalanobis_distance;
      callback_threshold = mahalanobis_threshold;
      callback_time = measurement_time;
    });

  robot_localization::Measurement initial_measurement;
  initial_measurement.topic_name_ = "test_stream";
  initial_measurement.time_ = rclcpp::Time(1, 0, RCL_ROS_TIME);
  initial_measurement.mahalanobis_thresh_ = std::numeric_limits<double>::max();
  initial_measurement.update_vector_ = std::vector<bool>(STATE_SIZE, false);
  initial_measurement.update_vector_[robot_localization::StateMemberX] = true;
  initial_measurement.measurement_ = Eigen::VectorXd::Zero(STATE_SIZE);
  initial_measurement.covariance_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.1;
  filter.processMeasurement(initial_measurement);

  robot_localization::Measurement rejected_measurement = initial_measurement;
  rejected_measurement.time_ = rclcpp::Time(2, 0, RCL_ROS_TIME);
  rejected_measurement.mahalanobis_thresh_ = 1.0;
  rejected_measurement.measurement_[robot_localization::StateMemberX] = 10.0;
  filter.processMeasurement(rejected_measurement);

  EXPECT_TRUE(callback_called);
  EXPECT_FALSE(callback_passed);
  EXPECT_EQ(callback_stream, "test_stream");
  EXPECT_GT(callback_distance, callback_threshold);
  EXPECT_EQ(callback_time, rejected_measurement.time_);
}

TEST(EkfTest, InnovationResultCallback) {
  Ekf filter;

  bool callback_called = false;
  size_t callback_count = 0;
  robot_localization::FilterBase::InnovationResult callback_result;

  filter.setInnovationResultCallback(
    [&](const robot_localization::FilterBase::InnovationResult & result)
    {
      callback_called = true;
      ++callback_count;
      callback_result = result;
    });

  robot_localization::Measurement initial_measurement;
  initial_measurement.topic_name_ = "test_stream";
  initial_measurement.time_ = rclcpp::Time(1, 0, RCL_ROS_TIME);
  initial_measurement.mahalanobis_thresh_ = std::numeric_limits<double>::max();
  initial_measurement.update_vector_ = std::vector<bool>(STATE_SIZE, false);
  initial_measurement.update_vector_[robot_localization::StateMemberX] = true;
  initial_measurement.measurement_ = Eigen::VectorXd::Zero(STATE_SIZE);
  initial_measurement.covariance_ =
    Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.1;
  filter.processMeasurement(initial_measurement);

  robot_localization::Measurement rejected_measurement = initial_measurement;
  rejected_measurement.time_ = rclcpp::Time(2, 0, RCL_ROS_TIME);
  rejected_measurement.mahalanobis_thresh_ = 1.0;
  rejected_measurement.measurement_[robot_localization::StateMemberX] = 10.0;
  filter.processMeasurement(rejected_measurement);

  EXPECT_TRUE(callback_called);
  EXPECT_GE(callback_count, 1u);
  EXPECT_EQ(callback_result.topic_name_, "test_stream");
  EXPECT_EQ(callback_result.measurement_time_, rejected_measurement.time_);
  ASSERT_EQ(callback_result.state_indices_.size(), 1u);
  EXPECT_EQ(
    callback_result.state_indices_.front(),
    static_cast<size_t>(robot_localization::StateMemberX));
  ASSERT_EQ(callback_result.measurement_.size(), 1);
  ASSERT_EQ(callback_result.predicted_measurement_.size(), 1);
  ASSERT_EQ(callback_result.innovation_.size(), 1);
  ASSERT_EQ(callback_result.measurement_covariance_diagonal_.size(), 1);
  ASSERT_EQ(callback_result.innovation_covariance_diagonal_.size(), 1);
  EXPECT_DOUBLE_EQ(callback_result.measurement_(0), 10.0);
  EXPECT_DOUBLE_EQ(callback_result.predicted_measurement_(0), 0.0);
  EXPECT_DOUBLE_EQ(callback_result.innovation_(0), 10.0);
  EXPECT_DOUBLE_EQ(callback_result.measurement_covariance_diagonal_(0), 0.1);
  EXPECT_GT(callback_result.innovation_covariance_diagonal_(0), 0.0);
  EXPECT_FALSE(callback_result.passed_);
  EXPECT_GT(
    callback_result.mahalanobis_distance_,
    callback_result.mahalanobis_threshold_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return ret;
}
