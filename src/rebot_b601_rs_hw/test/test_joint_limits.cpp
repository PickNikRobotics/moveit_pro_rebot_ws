// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
// No hardware plugin or CAN socket: execute the real URDF parser and the
// pinned driver's coordinate/bounds helpers used before enabling torque.

#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <hardware_interface/component_parser.hpp>
#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_base/robot_state/robot_state.hpp>
#include <robstride_hardware_interface/joint_coordinates.hpp>
#include <robstride_sdk/robstride_protocol.hpp>

#include <array>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>

namespace
{
class JointLimits : public testing::Test
{
protected:
  void SetUp() override
  {
    std::ifstream file(TEST_URDF);
    ASSERT_TRUE(file.good());
    const std::string urdf{ std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>() };
    const auto resources = hardware_interface::parse_control_resources_from_urdf(urdf);
    ASSERT_EQ(resources.size(), 1u);
    hardware = resources.front();
  }

  bool WithinBounds(const std::string& name, double position) const
  {
    const auto& limit = hardware.limits.at(name);
    const auto type = name == "joint1" || name == "joint2" || name == "joint3" ? robstride_sdk::ActuatorType::RS06 :
                                                                                 robstride_sdk::ActuatorType::RS00;
    const double quantum = 2.0 * robstride_sdk::GetActuatorLimits(type).position_max / 65535.0;
    const double wrapped =
        robstride_hardware_interface::WrapToWindow(position, 0.5 * (limit.min_position + limit.max_position));
    return robstride_hardware_interface::PositionWithinBounds(wrapped, limit.min_position, limit.max_position, quantum);
  }

  hardware_interface::HardwareInfo hardware;
};

TEST_F(JointLimits, ClosedJawFeedbackIsInsideDriverActivationBounds)
{
  const auto& jaw = hardware.limits.at("joint7");
  EXPECT_DOUBLE_EQ(jaw.min_position, -0.15);
  EXPECT_DOUBLE_EQ(jaw.max_position, 5.410520681182422);
  EXPECT_DOUBLE_EQ(jaw.max_velocity, 3.0);
  // At-rest and fast-close feedback measured on a bench unit, not closure targets.
  for (const double position : { -0.001, -0.0032596, -0.010545730590820312, -0.042, -0.120 })
  {
    EXPECT_TRUE(WithinBounds("joint7", position)) << position;
    EXPECT_NEAR(robstride_hardware_interface::JointToMotorTarget(position, position, 0, 1, false), position, 1e-14);
  }
}

TEST_F(JointLimits, JawStillRejectsOvertravelPastItsFeedbackAllowance)
{
  const double quantum =
      2.0 * robstride_sdk::GetActuatorLimits(robstride_sdk::ActuatorType::RS00).position_max / 65535.0;
  EXPECT_TRUE(WithinBounds("joint7", -0.15));
  EXPECT_TRUE(WithinBounds("joint7", -0.15 - 0.9 * quantum));
  EXPECT_FALSE(WithinBounds("joint7", -0.15 - 1.1 * quantum));
  EXPECT_FALSE(WithinBounds("joint7", -0.151));
  EXPECT_FALSE(WithinBounds("joint7", -0.17));
  EXPECT_TRUE(WithinBounds("joint7", 5.410520681182422));
  EXPECT_FALSE(WithinBounds("joint7", 5.410520681182422 + 1.1 * quantum));
  EXPECT_FALSE(WithinBounds("joint7", std::numeric_limits<double>::quiet_NaN()));
}

TEST_F(JointLimits, MoveItAcceptsMeasuredContactAndRejectsTrueJawOvertravelWithoutClamping)
{
  // Build a real model, not a text assertion over the Xacro or copied predicate.
  std::ifstream urdf_file(TEST_URDF);
  std::ifstream srdf_file(TEST_SRDF);
  ASSERT_TRUE(urdf_file.good());
  ASSERT_TRUE(srdf_file.good());
  const std::string urdf_text{ std::istreambuf_iterator<char>(urdf_file), std::istreambuf_iterator<char>() };
  const std::string srdf_text{ std::istreambuf_iterator<char>(srdf_file), std::istreambuf_iterator<char>() };
  const auto urdf = urdf::parseURDF(urdf_text);
  ASSERT_NE(urdf, nullptr);
  auto srdf = std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf->initString(*urdf, srdf_text));
  auto model = std::make_shared<moveit_pro::base::RobotModel>(urdf, srdf);
  moveit_pro::base::RobotState state(model);
  for (const double position :
       { -0.15, -0.120, -0.042, -0.001, -0.0032596, -0.010545730590820312, 0.0, 5.410520681182422 })
  {
    state.setVariablePosition("joint7", position);
    state.update();
    EXPECT_TRUE(state.satisfiesBounds(model->getJointModelGroup("gripper"), 0.0)) << position;
    EXPECT_DOUBLE_EQ(state.getVariablePosition("joint7"), position);
    for (const auto& finger : { "gripper_joint1", "gripper_joint2" })
    {
      EXPECT_TRUE(state.satisfiesBounds(model->getJointModel(finger), 0.0)) << finger << ": " << position;
      EXPECT_DOUBLE_EQ(state.getVariablePosition(finger), 0.008 * position);
    }
  }
  for (const double position : { -0.1501, -0.151, 5.410620681182422 })
  {
    state.setVariablePosition("joint7", position);
    EXPECT_FALSE(state.satisfiesBounds(model->getJointModelGroup("gripper"), 0.0)) << position;
    EXPECT_DOUBLE_EQ(state.getVariablePosition("joint7"), position);
  }
}

TEST_F(JointLimits, AllSixArmLimitsAndTheirFeedbackToleranceAreUnchanged)
{
  const std::array<std::array<double, 2>, 6> bounds{ {
      { -2.530727415392, 2.530727415392 },
      { -0.02, 3.543018381548 },
      { -0.01, 4.171336912266 },
      { -1.850049007114, 1.500983156715 },
      { -1.762782544514, 1.884955592154 },
      { -3.054326190990, 3.054326190990 },
  } };
  for (std::size_t i = 0; i < bounds.size(); ++i)
  {
    const auto name = "joint" + std::to_string(i + 1);
    const auto& limit = hardware.limits.at(name);
    EXPECT_DOUBLE_EQ(limit.min_position, bounds[i][0]) << name;
    EXPECT_DOUBLE_EQ(limit.max_position, bounds[i][1]) << name;
    EXPECT_DOUBLE_EQ(limit.max_velocity, 0.6) << name;
    EXPECT_TRUE(WithinBounds(name, bounds[i][0])) << name;
    EXPECT_TRUE(WithinBounds(name, bounds[i][1])) << name;
    EXPECT_FALSE(WithinBounds(name, bounds[i][0] - 0.001)) << name;
    EXPECT_FALSE(WithinBounds(name, bounds[i][1] + 0.001)) << name;
  }
}
}  // namespace
