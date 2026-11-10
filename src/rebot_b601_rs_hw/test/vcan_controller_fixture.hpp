// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
// The admittance and velocity-force controller headers declare conflicting
// DiffInverseKinematicsFn aliases, so the pose jog lives in its own unit.

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <rclcpp/node_options.hpp>
#include <ros2_control_utils/ros2_control_utils.hpp>
#include <string>
#include <utility>

namespace rebot_vcan_test
{
// Load the actual stock YAML instead of replicating controller settings here.
template <class Controller>
struct Configured : Controller
{
  explicit Configured(std::string file) : file_(std::move(file))
  {
  }
  rclcpp::NodeOptions define_custom_node_options() const override
  {
    auto options = Controller::define_custom_node_options();
    options.arguments({ "--ros-args", "--params-file", file_ });
    return options;
  }
  std::string file_;
};

struct PoseJog
{
  std::shared_ptr<controller_interface::ControllerInterface> controller;
  moveit_pro_controllers::ControllerWithRobotModel* model = nullptr;
};

PoseJog MakePoseJog(const std::string& params_file);
}  // namespace rebot_vcan_test
