// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <array>
#include <fstream>
#include <iterator>
#include <map>
#include <random>
#include <string>

#include "kdl_gravity_compensation_controller/gravity_model.hpp"

namespace
{
std::string ReadFile(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("Cannot read " + path);
  return { std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>() };
}

struct StockGravity : testing::Test
{
  void SetUp() override
  {
    urdf = ReadFile(TEST_URDF);
    ASSERT_TRUE(kdl_parser::treeFromString(urdf, tree));
    parameters = YAML::LoadFile(CONTROL_YAML)["gravity_compensation_controller"]["ros__parameters"];
    for (const auto& [name, element] : tree.getSegments())
    {
      const auto& joint = element.segment.getJoint();
      if (joint.getType() != KDL::Joint::None)
        joint_indices.emplace(joint.getName(), element.q_nr);
    }
  }

  KDL::Vector PayloadCom() const
  {
    const auto com = parameters["payload_com"].as<std::vector<double>>();
    return { com.at(0), com.at(1), com.at(2) };
  }

  std::string urdf;
  KDL::Tree tree;
  YAML::Node parameters;
  std::map<std::string, unsigned int> joint_indices;
};

TEST_F(StockGravity, PayloadRepresentsOnlyTheTwoSuppliedFingers)
{
  double mass = 0.0;
  KDL::Vector first_moment = KDL::Vector::Zero();
  for (const auto& name : { "gripper_left", "gripper_right" })
  {
    const auto& segment = tree.getSegment(name)->second.segment;
    mass += segment.getInertia().getMass();
    first_moment += segment.getInertia().getMass() * (segment.pose(0.0) * segment.getInertia().getCOG());
  }
  EXPECT_DOUBLE_EQ(mass, 0.1504);
  EXPECT_DOUBLE_EQ(parameters["payload_mass"].as<double>(), mass);
  for (unsigned int axis = 0; axis < 3; ++axis)
    EXPECT_NEAR(PayloadCom()[axis], first_moment[axis] / mass, 1e-14);
  EXPECT_EQ(parameters["gains"].as<std::vector<double>>(), (std::vector<double>(6, 1.0)));
  EXPECT_EQ(parameters["offsets"].as<std::vector<double>>(), (std::vector<double>(6, 0.0)));
}

TEST_F(StockGravity, SerialChainPlusPayloadMatchesWholeTreeGravity)
{
  const auto names = parameters["joints"].as<std::vector<std::string>>();
  ASSERT_EQ(names.size(), 6u);
  const auto configured_gravity = parameters["gravity_vector"].as<std::vector<double>>();
  const KDL::Vector gravity(configured_gravity.at(0), configured_gravity.at(1), configured_gravity.at(2));
  kdl_gravity_compensation_controller::GravityModel chain(urdf, parameters["root_link"].as<std::string>(),
                                                          parameters["tip_link"].as<std::string>(), gravity,
                                                          parameters["payload_mass"].as<double>(), PayloadCom());
  EXPECT_EQ(chain.joint_names(), names);
  // Separate whole-tree inverse dynamics includes both moving finger branches.
  // Zero joint velocity/acceleration makes the result pure gravity.
  KDL::TreeIdSolver_RNE whole_tree(tree, gravity);
  KDL::JntArray q(tree.getNrOfJoints()), zero(tree.getNrOfJoints()), torque(tree.getNrOfJoints());
  KDL::SetToZero(zero);
  KDL::WrenchMap external;
  for (const auto& [name, element] : tree.getSegments())
    external.emplace(name, KDL::Wrench::Zero());
  KDL::JntArray arm_q(6), arm_torque(6);
  std::mt19937 random(601);
  std::uniform_real_distribution<double> angles(-2.0, 2.0);
  for (unsigned int sample = 0; sample < 50; ++sample)
  {
    KDL::SetToZero(q);
    for (unsigned int joint = 0; joint < 6; ++joint)
    {
      // Include HOME, Raised, then deterministic varied orientations.
      const std::array<double, 6> raised{ 0.0, 0.65, 0.85, 0.3, 0.0, 0.0 };
      arm_q(joint) = sample == 0 ? 0.0 : sample == 1 ? raised[joint] : angles(random);
      q(joint_indices.at(names[joint])) = arm_q(joint);
    }
    ASSERT_TRUE(chain.Compute(arm_q, arm_torque));
    for (const double jaw : { 0.0, 2.705260340591211, 5.410520681182422 })
    {
      q(joint_indices.at("joint7")) = jaw;
      q(joint_indices.at("gripper_joint1")) = 0.008 * jaw;
      q(joint_indices.at("gripper_joint2")) = 0.008 * jaw;
      ASSERT_EQ(whole_tree.CartToJnt(q, zero, zero, external, torque), 0);
      for (unsigned int joint = 0; joint < 6; ++joint)
      {
        // Rounded nominal mounting rotations cause <0.16 um CoM drift over
        // the full jaw travel: <=0.1504*9.81*0.16e-6 = 2.37e-7 Nm.
        const double tolerance = jaw == 0.0 ? 1e-11 : 2.4e-7;
        EXPECT_NEAR(arm_torque(joint), torque(joint_indices.at(names[joint])), tolerance)
            << "sample=" << sample << " jaw=" << jaw << " joint=" << names[joint];
      }
    }
  }
}
}  // namespace
