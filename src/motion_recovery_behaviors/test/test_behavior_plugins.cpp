// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <motion_recovery_behaviors/plan_to_joint_goal_from_bounded_start.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_pro_test_utils/ros_test.hpp>
#include <pluginlib/class_loader.hpp>

#include <limits>

namespace
{
class RecoveryPlugins : public moveit_pro::test_utils::RosTest
{
protected:
  void SetUp() override
  {
    auto node = std::make_shared<rclcpp::Node>("test_motion_recovery_plugins");
    context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false);
    const auto urdf = urdf::parseURDF(R"(<robot name="fixture">
      <link name="base"/><link name="arm"/><link name="tool"/>
      <joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>
        <axis xyz="0 1 0"/><limit lower="-0.2" upper="1.0" effort="2" velocity="0.6"/></joint>
      <joint name="spin" type="continuous"><parent link="arm"/><child link="tool"/>
        <axis xyz="0 0 1"/><limit effort="1" velocity="0.6"/></joint>
      </robot>)");
    ASSERT_TRUE(urdf);
    auto srdf = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf, R"(<robot name="fixture"><group name="arm">
      <chain base_link="base" tip_link="tool"/></group></robot>)"));
    context->robot_model = std::make_shared<moveit_pro::base::RobotModel>(urdf, srdf);
    loader.createUniqueInstance("motion_recovery_behaviors::MotionRecoveryBehaviorsLoader")
        ->registerBehaviors(factory, context);
  }

  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader{
    "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase"
  };
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context;
  BT::BehaviorTreeFactory factory;
};

TEST_F(RecoveryPlugins, RegistersOnlyTheThreeHelpersAndInstantiatesTheirNativePorts)
{
  for (const auto* id : { "ClampRobotStateToBounds", "PlanToJointGoalFromBoundedStart", "AnchorTrajectoryToCommand" })
  {
    SCOPED_TRACE(id);
    ASSERT_TRUE(factory.manifests().contains(id));
    BT::NodeConfig config;
    config.blackboard = BT::Blackboard::create();
    EXPECT_NO_THROW(factory.instantiateTreeNode("helper", id, config));
  }
  EXPECT_FALSE(factory.manifests().contains("RecordRebotTrajectory"));
  const auto native = moveit_pro::behaviors::PlanToJointGoal::providedPorts();
  const auto adapted = motion_recovery_behaviors::PlanToJointGoalFromBoundedStart::providedPorts();
  ASSERT_EQ(adapted.size(), native.size());
  for (const auto& [name, port] : native)
  {
    ASSERT_TRUE(adapted.contains(name));
    EXPECT_EQ(adapted.at(name).direction(), port.direction());
    EXPECT_EQ(adapted.at(name).type(), port.type());
  }
}

TEST_F(RecoveryPlugins, ClampsBothBoundsAndPreservesContinuousJointAndStateFields)
{
  for (const auto& [position, expected] : { std::pair{ -0.5, -0.2 }, { 0.4, 0.4 }, { 1.5, 1.0 } })
  {
    auto tree = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Test">
      <Action ID="ClampRobotStateToBounds" robot_state="{input}" bounded_state="{output}"/>
      </BehaviorTree></root>)");
    moveit_msgs::msg::RobotState input;
    input.joint_state.name = { "spin", "hinge" };
    input.joint_state.position = { 10.0, position };
    input.joint_state.velocity = { 0.1, 0.2 };
    input.is_diff = true;
    tree.rootBlackboard()->set("input", input);
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    const auto output = tree.rootBlackboard()->get<moveit_msgs::msg::RobotState>("output");
    EXPECT_THAT(output.joint_state.position, testing::ElementsAre(10.0, expected));
    EXPECT_EQ(output.joint_state.name, input.joint_state.name);
    EXPECT_EQ(output.joint_state.velocity, input.joint_state.velocity);
    EXPECT_EQ(output.is_diff, input.is_diff);
    EXPECT_EQ(tree.rootBlackboard()->get<moveit_msgs::msg::RobotState>("input"), input);
  }
}

TEST_F(RecoveryPlugins, MissingMalformedUnknownAndNonfiniteStatesFailWithoutOutput)
{
  for (const auto* scenario : { "missing", "mismatch", "unknown", "nonfinite", "no_model" })
  {
    SCOPED_TRACE(scenario);
    auto tree = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Test">
      <Action ID="ClampRobotStateToBounds" robot_state="{input}" bounded_state="{output}"/>
      </BehaviorTree></root>)");
    moveit_msgs::msg::RobotState input;
    input.joint_state.name = { std::string(scenario) == "unknown" ? "unknown" : "hinge" };
    if (std::string(scenario) != "mismatch")
      input.joint_state.position = { std::string(scenario) == "nonfinite" ? std::numeric_limits<double>::quiet_NaN() :
                                                                            0.0 };
    if (std::string(scenario) != "missing")
      tree.rootBlackboard()->set("input", input);
    if (std::string(scenario) == "no_model")
      context->robot_model.reset();
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
    moveit_msgs::msg::RobotState unused;
    EXPECT_FALSE(tree.rootBlackboard()->get("output", unused));
    EXPECT_FALSE(context->logger->consumeErrorLogBuffer().empty());
  }
}
}  // namespace
