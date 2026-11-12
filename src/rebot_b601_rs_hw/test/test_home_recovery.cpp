// Copyright 2026 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// Derived from example_ws test_jaw_legs.cpp at c7422c8853e495f0e481daa639c2919be24d73f9.
// Original HOME and native-planner cases retained; office jaw/pose cases stay in Boston.
#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>
#include <motion_recovery_behaviors/anchor_trajectory_to_command.hpp>
#include <motion_recovery_behaviors/clamp_robot_state_to_bounds.hpp>
#include <motion_recovery_behaviors/plan_to_joint_goal_from_bounded_start.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_base/planning_scene/planning_scene.hpp>
#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_base/robot_state/robot_state.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace
{
using Positions = std::map<std::string, double>;
using RobotState = moveit_msgs::msg::RobotState;
using PlanningScene = moveit_msgs::msg::PlanningScene;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
const std::vector<std::string> kArm{ "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
const std::map<std::string, std::vector<std::string>> kGroups{
  { "manipulator", kArm },
  { "manipulator_and_gripper", { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7" } }
};
const Positions kHomePose{ { "joint1", 0.0 }, { "joint2", 0.0 }, { "joint3", 0.0 },
                           { "joint4", 0.0 }, { "joint5", 0.0 }, { "joint6", 0.0 } };
const Positions kSaggedHome{ { "joint1", 0.0 },     { "joint2", 0.0 }, { "joint3", 0.0 },
                             { "joint4", -0.0155 }, { "joint5", 0.0 }, { "joint6", 0.0 } };
// Off HOME with joint3 below its lower bound, far enough for self-collision.
const Positions kDeepSag{ { "joint1", 0.0 }, { "joint2", 0.0 }, { "joint3", -0.08 },
                          { "joint4", 0.5 }, { "joint5", 0.3 }, { "joint6", 0.0 } };
constexpr double kArmSpeedLimit = 0.6;

std::string readFile(const std::string& path)
{
  std::ifstream stream(path);
  return { std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>() };
}

RobotState robotState(const Positions& positions)
{
  RobotState state;
  for (const auto& [joint, position] : positions)
  {
    state.joint_state.name.push_back(joint);
    state.joint_state.position.push_back(position);
  }
  return state;
}

Positions positionsOf(const sensor_msgs::msg::JointState& state)
{
  Positions positions;
  for (std::size_t i = 0; i < state.name.size(); ++i)
    positions[state.name[i]] = state.position.at(i);
  return positions;
}

Positions withJaw(Positions positions, const double jaw)
{
  positions["joint7"] = jaw;
  return positions;
}

double seconds(const builtin_interfaces::msg::Duration& duration)
{
  return rclcpp::Duration(duration).seconds();
}

// Deterministic planner seam. The real clamping and anchoring Behaviors execute;
// native planning is separately retained below behind its normal license gate.
JointTrajectory linearTrajectory(const std::vector<std::string>& joints, const Positions& start, const Positions& goal,
                                 const double scale)
{
  double duration = 0.5;
  for (const auto& joint : joints)
    duration = std::max(duration, std::abs(goal.at(joint) - start.at(joint)) / (kArmSpeedLimit * scale));
  JointTrajectory trajectory;
  trajectory.joint_names = joints;
  constexpr int kSegments = 10;
  for (int i = 0; i <= kSegments; ++i)
  {
    const double s = static_cast<double>(i) / kSegments;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(s * duration);
    for (const auto& joint : joints)
    {
      const double distance = goal.at(joint) - start.at(joint);
      point.positions.push_back(start.at(joint) + s * distance);
      point.velocities.push_back(i == 0 || i == kSegments ? 0.0 : distance / duration);
      point.accelerations.push_back(0.0);
    }
    trajectory.points.push_back(point);
  }
  return trajectory;
}

std::vector<double> column(const JointTrajectory& trajectory, const std::string& joint)
{
  const auto index =
      static_cast<std::size_t>(std::ranges::find(trajectory.joint_names, joint) - trajectory.joint_names.begin());
  std::vector<double> positions;
  for (const auto& point : trajectory.points)
    positions.push_back(point.positions.at(index));
  return positions;
}

class MeasuredScene final : public moveit_pro::behaviors::ClientInterfaceBase<moveit_pro::behaviors::GetPlanningScene>
{
public:
  MeasuredScene(const PlanningScene& scene, const Positions& measured) : scene_(scene), measured_(measured)
  {
  }
  void initialize(const std::string&, std::chrono::duration<double>, std::chrono::duration<double>) override
  {
  }
  bool waitForServiceServer() const override
  {
    return true;
  }
  tl::expected<moveit_pro::behaviors::GetPlanningScene::Response, std::string>
  syncSendRequest(const moveit_pro::behaviors::GetPlanningScene::Request&) override
  {
    moveit_pro::behaviors::GetPlanningScene::Response response;
    response.scene = scene_;
    response.scene.robot_state = robotState(measured_);
    return response;
  }
  void cancelRequest() override
  {
  }

private:
  const PlanningScene& scene_;
  const Positions& measured_;
};

class HomeRecovery : public testing::Test
{
protected:
  void SetUp() override
  {
    const auto urdf = urdf::parseURDF(readFile(TEST_URDF));
    ASSERT_TRUE(urdf);
    auto srdf = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf, readFile(STOCK_SHARE "/config/moveit/rebot.srdf")));
    context->robot_model = std::make_shared<moveit_pro::base::RobotModel>(urdf, srdf);
    for (const auto& entry : YAML::LoadFile(STOCK_SHARE "/config/moveit/joint_limits.yaml")["joint_limits"])
    {
      const auto name = entry.first.as<std::string>();
      auto* joint = context->robot_model->getJointModel(name);
      ASSERT_NE(joint, nullptr);
      auto bounds = joint->getVariableBounds().front();
      if (entry.second["has_acceleration_limits"].as<bool>(false))
      {
        bounds.acceleration_bounded = true;
        bounds.max_acceleration = entry.second["max_acceleration"].as<double>();
        bounds.min_acceleration = -bounds.max_acceleration;
      }
      joint->setVariableBounds(name, bounds);
    }
    moveit_pro::behaviors::registerBehavior<motion_recovery_behaviors::AnchorTrajectoryToCommand>(
        factory, "AnchorTrajectoryToCommand", context);
    moveit_pro::behaviors::registerBehavior<motion_recovery_behaviors::ClampRobotStateToBounds>(
        factory, "ClampRobotStateToBounds", context);
    factory.registerSimpleAction("RetrieveWaypoint",
                                 [](BT::TreeNode& node) {
                                   EXPECT_EQ(node.getInput<std::string>("waypoint_name").value(), "Zeroed Rest");
                                   node.setOutput("waypoint_joint_state", robotState(kHomePose));
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::InputPort<std::string>("joint_group_name"),
                                   BT::InputPort<std::string>("waypoint_name"),
                                   BT::OutputPort<RobotState>("waypoint_joint_state") });
    factory.registerSimpleAction("GetCurrentPlanningScene",
                                 [](BT::TreeNode& node) {
                                   node.setOutput("planning_scene_msg", PlanningScene{});
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::OutputPort<PlanningScene>("planning_scene_msg") });
    factory.registerSimpleAction(
        "GetJointState",
        [this](BT::TreeNode& node) {
          Positions group;
          for (const auto& joint : kGroups.at(node.getInput<std::string>("planning_group_name").value()))
            group[joint] = measured.at(joint);
          const auto state = robotState(group);
          node.setOutput("robot_joint_state", state);
          node.setOutput("joint_state", state.joint_state);
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<PlanningScene>("planning_scene"), BT::InputPort<std::string>("planning_group_name"),
          BT::OutputPort<RobotState>("robot_joint_state"),
          BT::OutputPort<sensor_msgs::msg::JointState>("joint_state") });
    factory.registerSimpleAction(
        "GeneratePointToPointTrajectory",
        [this](BT::TreeNode& node) {
          ++fallbacks;
          const auto& joints = kGroups.at(node.getInput<std::string>("planning_group_name").value());
          const auto start = positionsOf(node.getInput<RobotState>("start_state").value().joint_state);
          auto goal = start;
          for (const auto& [joint, position] :
               positionsOf(node.getInput<RobotState>("target_state").value().joint_state))
            goal[joint] = position;
          const auto scale = node.getInput<double>("velocity_scale_factor").value();
          EXPECT_DOUBLE_EQ(node.getInput<double>("acceleration_scale_factor").value(), scale);
          node.setOutput("joint_trajectory_msg", linearTrajectory(joints, start, goal, scale));
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<std::string>("planning_group_name"), BT::InputPort<RobotState>("start_state"),
          BT::InputPort<RobotState>("target_state"), BT::InputPort<double>("velocity_scale_factor"),
          BT::InputPort<double>("acceleration_scale_factor"), BT::InputPort<double>("jerk_scale_factor"),
          BT::InputPort<double>("trajectory_sampling_rate"), BT::InputPort<double>("start_time"),
          BT::OutputPort<JointTrajectory>("joint_trajectory_msg") });
    factory.registerSimpleAction(
        "PlanToJointGoalFromBoundedStart",
        [this](BT::TreeNode& node) {
          ++plans;
          if (planner_refuses_start)
            return BT::NodeStatus::FAILURE;
          auto state = robotState(measured);
          const auto bounded = motion_recovery_behaviors::clampToBounds(*context->robot_model, state.joint_state);
          if (!bounded)
            return BT::NodeStatus::FAILURE;
          const auto start = positionsOf(state.joint_state);
          auto goal = start;
          for (const auto& [joint, position] : positionsOf(node.getInput<RobotState>("joint_goal").value().joint_state))
            goal[joint] = position;
          const auto scale = node.getInput<double>("velocity_scale_factor").value();
          EXPECT_DOUBLE_EQ(node.getInput<double>("acceleration_scale_factor").value(), scale);
          node.setOutput("joint_trajectory_msg", linearTrajectory(kArm, start, goal, scale));
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<RobotState>("joint_goal"), BT::InputPort<std::string>("planning_group_name"),
          BT::InputPort<double>("velocity_scale_factor"), BT::InputPort<double>("acceleration_scale_factor"),
          BT::InputPort<std::string>("collision_check_params"),
          BT::OutputPort<JointTrajectory>("joint_trajectory_msg") });
    factory.registerSimpleAction(
        "ValidateTrajectory",
        [this](BT::TreeNode& node) {
          if (++validations == refused_validation)
            return BT::NodeStatus::FAILURE;
          const auto& group = kGroups.at(node.getInput<std::string>("planning_group_name").value());
          const auto names = node.getInput<JointTrajectory>("joint_trajectory_msg").value().joint_names;
          EXPECT_DOUBLE_EQ(node.getInput<double>("joint_space_step").value(), 0.02);
          EXPECT_DOUBLE_EQ(node.getInput<double>("cartesian_space_step").value(), 0.005);
          return std::ranges::all_of(names, [&](const auto& joint) { return std::ranges::count(group, joint) > 0; }) ?
                     BT::NodeStatus::SUCCESS :
                     BT::NodeStatus::FAILURE;
        },
        { BT::InputPort<PlanningScene>("planning_scene_msg"), BT::InputPort<std::string>("planning_group_name"),
          BT::InputPort<JointTrajectory>("joint_trajectory_msg"), BT::InputPort<double>("joint_space_step"),
          BT::InputPort<double>("cartesian_space_step") });
    factory.registerSimpleAction(
        "ExecuteTrajectory",
        [this](BT::TreeNode& node) {
          EXPECT_EQ(node.getInput<std::string>("execution_pipeline").value(), "jtac");
          EXPECT_EQ(node.getInput<std::string>("controller_action_name").value(),
                    "/joint_trajectory_admittance_controller/follow_joint_trajectory");
          EXPECT_DOUBLE_EQ(node.getInput<double>("goal_duration_tolerance").value(), -1.0);
          const auto trajectory = node.getInput<JointTrajectory>("joint_trajectory_msg").value();
          executed.push_back(trajectory);
          if (execution_refuses)
            return BT::NodeStatus::FAILURE;
          for (std::size_t j = 0; j < trajectory.joint_names.size(); ++j)
            measured[trajectory.joint_names[j]] = trajectory.points.back().positions.at(j);
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<std::string>("execution_pipeline"), BT::InputPort<std::string>("controller_action_name"),
          BT::InputPort<JointTrajectory>("joint_trajectory_msg"), BT::InputPort<double>("goal_duration_tolerance") });
    factory.registerSimpleAction("SwitchController",
                                 [this](BT::TreeNode& node) {
                                   ++switches;
                                   EXPECT_EQ(node.getInput<std::string>("activate_controllers").value(),
                                             "joint_trajectory_admittance_controller");
                                   EXPECT_EQ(node.getInput<std::string>("automatic_deactivation").value(), "true");
                                   EXPECT_EQ(node.getInput<std::string>("strictness").value(), "2");
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::InputPort<std::string>("activate_controllers"),
                                   BT::InputPort<std::string>("automatic_deactivation"),
                                   BT::InputPort<std::string>("strictness") });
    factory.registerSimpleAction(
        "UnpackJointStateMessage",
        [](BT::TreeNode& node) {
          node.setOutput("position", node.getInput<sensor_msgs::msg::JointState>("joint_state").value().position);
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<sensor_msgs::msg::JointState>("joint_state"),
          BT::OutputPort<std::vector<double>>("position") });
    factory.registerSimpleAction(
        "GetElementOfVector",
        [](BT::TreeNode& node) {
          const auto vector = node.getInput<std::vector<double>>("vector_in").value();
          node.setOutput("element", vector.at(static_cast<std::size_t>(node.getInput<int>("index").value())));
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<std::vector<double>>("vector_in"), BT::InputPort<int>("index"),
          BT::OutputPort<double>("element") });
    factory.registerSimpleAction("CreateRobotState",
                                 [](BT::TreeNode& node) {
                                   const auto names = node.getInput<std::vector<std::string>>("joint_names").value();
                                   const auto values = node.getInput<std::vector<double>>("positions").value();
                                   Positions state;
                                   for (std::size_t i = 0; i < names.size(); ++i)
                                     state[names[i]] = values.at(i);
                                   node.setOutput("joint_state_msg", robotState(state));
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::InputPort<std::vector<std::string>>("joint_names"),
                                   BT::InputPort<std::vector<double>>("positions"),
                                   BT::OutputPort<RobotState>("joint_state_msg") });
    factory.registerSimpleAction(
        "WaitForDuration",
        [this](BT::TreeNode& node) {
          const auto duration = node.getInput<double>("delay_duration").value();
          if (duration == 0.5)
            home_tail.push_back("settle");
          else
          {
            const auto poses = YAML::LoadFile(PACKAGE_DIR "/config/relax_safe_poses.yaml");
            EXPECT_DOUBLE_EQ(duration, poses["relax_safe_poses"][0]["stillness"]["window_s"].as<double>() + 0.1);
            EXPECT_DOUBLE_EQ(measured.at("joint7"), 0.0);
            home_tail.push_back("dwell");
          }
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<double>("delay_duration") });
    factory.registerSimpleAction(
        "CallTriggerService",
        [this](BT::TreeNode& node) {
          const auto service = node.getInput<std::string>("service_name").value();
          if (service == "/robstride_hardware_interface/engage")
          {
            EXPECT_DOUBLE_EQ(node.getInput<double>("wait_for_server_available_timeout").value(), 3.0);
            EXPECT_DOUBLE_EQ(node.getInput<double>("response_timeout").value(), 3.0);
            ++engages;
            return engage_refuses ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
          }
          if (service == "/robstride_hardware_interface/relax")
          {
            EXPECT_DOUBLE_EQ(node.getInput<double>("wait_for_server_available_timeout").value(), 3.0);
            EXPECT_DOUBLE_EQ(node.getInput<double>("response_timeout").value(), 3.0);
            home_tail.push_back("relax");
            return relax_refuses ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
          }
          if (service == "/rebot_leader_bridge/idle")
          {
            EXPECT_DOUBLE_EQ(node.getInput<double>("wait_for_server_available_timeout").value(), 1.0);
            EXPECT_DOUBLE_EQ(node.getInput<double>("response_timeout").value(), 1.0);
            ++idle_checks;
            return idle_refuses ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
          }
          EXPECT_EQ(service, "/rebot_leader_bridge/home_verified");
          EXPECT_DOUBLE_EQ(node.getInput<double>("wait_for_server_available_timeout").value(), 1.0);
          EXPECT_DOUBLE_EQ(node.getInput<double>("response_timeout").value(), 1.0);
          for (const auto& joint : kArm)
            EXPECT_LE(std::abs(measured.at(joint)), 0.08);
          home_tail.push_back("verify");
          return verification_refuses ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<std::string>("service_name"), BT::InputPort<double>("wait_for_server_available_timeout"),
          BT::InputPort<double>("response_timeout") });
    factory.registerSimpleAction("CloseJaw",
                                 [this](BT::TreeNode& node) {
                                   EXPECT_EQ(node.getInput<std::string>("joint_group_name").value(), "gripper");
                                   const auto goal = node.getInput<RobotState>("target_joint_state").value();
                                   EXPECT_EQ(goal.joint_state.name, (std::vector<std::string>{ "joint7" }));
                                   for (const auto& joint : kArm)
                                     EXPECT_LE(std::abs(measured.at(joint)), 0.08);
                                   home_tail.push_back("close");
                                   if (close_refuses)
                                     return BT::NodeStatus::FAILURE;
                                   measured["joint7"] = goal.joint_state.position.at(0);
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::InputPort<RobotState>("target_joint_state"),
                                   BT::InputPort<std::string>("joint_group_name") });
    // Keep the shipped Close Gripper tree; only its external motion leaf is a seam.
    factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4">
      <BehaviorTree ID="Move to Joint State"><Action ID="CloseJaw"
        target_joint_state="{target_joint_state}" joint_group_name="{joint_group_name}"/></BehaviorTree></root>)");
    // The gain subtree is exercised by test_gravity_mode_execution and vcan; here
    // only its place in the HOME wrapper and in Relax at HOME (after engage and the
    // switch, before motion).
    factory.registerSimpleAction("SpringRestore", [this](BT::TreeNode&) {
      spring_restores.push_back({ engages, switches, executed.size() });
      return BT::NodeStatus::SUCCESS;
    });
    factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4">
      <BehaviorTree ID="reBot Set Spring Gains"><Action ID="SpringRestore"/></BehaviorTree></root>)");
    for (const auto* objective : { "rebot_move_to_arm_waypoint.xml", "rebot_check_leader_idle.xml" })
      factory.registerBehaviorTreeFromFile(std::string(PACKAGE_DIR "/objectives/") + objective);
    for (const auto* objective : { "return_rebot_leader_home.xml", "relax_at_home.xml" })
      factory.registerBehaviorTreeFromFile(std::string(GENERATED_OBJECTIVES "/") + objective);
    factory.registerBehaviorTreeFromFile(BOSTON_SHARE "/objectives/rebot_notify_leader_home.xml");
  }

  void useNoLeaderNotification()
  {
    factory.registerBehaviorTreeFromFile(PACKAGE_DIR "/objectives/rebot_notify_leader_home.xml");
  }

  BT::NodeStatus run(const std::string& subtree_call, const Positions& commanded = {})
  {
    tree = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)" + subtree_call +
                                      "</BehaviorTree></root>");
    tree.rootBlackboard()->set("commanded_state", robotState(commanded));
    return tree.tickWhileRunning();
  }

  void useNativePlanners()
  {
    factory.unregisterBuilder("PlanToJointGoalFromBoundedStart");
    registerPlanner<moveit_pro::behaviors::PlanToJointGoal>("PlanToJointGoal");
    registerPlanner<motion_recovery_behaviors::PlanToJointGoalFromBoundedStart>("PlanToJointGoalFromBoundedStart");
    moveit_pro::base::planning_scene::PlanningScene(context->robot_model).getPlanningSceneMsg(scene);
  }

  template <typename Planner>
  void registerPlanner(const std::string& id)
  {
    factory.registerBuilder(BT::CreateManifest<Planner>(id), [this](const std::string& name,
                                                                    const BT::NodeConfig& config) {
      return std::make_unique<Planner>(name, config, context, std::make_unique<MeasuredScene>(scene, measured));
    });
  }

  BT::NodeStatus planHome(const std::string& planner)
  {
    tree = factory.createTreeFromText(
        R"(<root BTCPP_format="4"><BehaviorTree ID="Test"><Action ID=")" + planner +
        R"(" joint_goal="{goal}" planning_group_name="manipulator" velocity_scale_factor="1.0" acceleration_scale_factor="1.0"
        collision_check_params="[{override_scene_link_padding: true, link_padding: 0.0}]" joint_trajectory_msg="{plan}"/></BehaviorTree></root>)");
    tree.rootBlackboard()->set("goal", robotState(kHomePose));
    return tree.tickWhileRunning();
  }

  bool withinBounds(const Positions& positions) const
  {
    moveit_pro::base::RobotState state(context->robot_model);
    state.setToDefaultValues();
    for (const auto& [joint, position] : positions)
      state.setVariablePosition(joint, position);
    return std::ranges::all_of(positions, [&](const auto& joint) {
      return state.satisfiesBounds(context->robot_model->getJointModel(joint.first), 0.0);
    });
  }

  Positions committed()
  {
    return positionsOf(tree.rootBlackboard()->get<RobotState>("commanded_state").joint_state);
  }
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context =
      std::make_shared<moveit_pro::behaviors::BehaviorContext>(std::make_shared<rclcpp::Node>("test_home_recovery"),
                                                               false);
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  Positions measured;
  PlanningScene scene;
  std::vector<JointTrajectory> executed;
  int validations = 0, refused_validation = -1, plans = 0, fallbacks = 0, switches = 0;
  bool planner_refuses_start = false, execution_refuses = false;
  bool verification_refuses = false, close_refuses = false, engage_refuses = false;
  int engages = 0, idle_checks = 0;
  // engages, switches and executed trajectories seen when the spring gains were restored.
  std::vector<std::tuple<int, int, std::size_t>> spring_restores;
  bool relax_refuses = false, idle_refuses = false;
  std::vector<std::string> home_tail;
};

const std::string kHome =
    R"(<SubTree ID="reBot Move to Arm Waypoint" waypoint_name="Zeroed Rest" velocity_scale_factor="1.0"
  acceleration_scale_factor="1.0" commanded_state="{commanded_state}"/>)";
const std::string kReturn = R"(<SubTree ID="Return reBot Leader Home"/>)";

TEST_F(HomeRecovery, NativePlannerRefusesADeepSagThatTheBoundedStartPlansFromItsBound)
{
  if (std::getenv("MOVEIT_LICENSE_KEY") == nullptr)
    GTEST_SKIP() << "Native planning needs a MoveIt Pro license in MOVEIT_LICENSE_KEY.";
  useNativePlanners();
  measured = withJaw(kDeepSag, 2.0);
  ASSERT_FALSE(withinBounds(measured));
  EXPECT_EQ(planHome("PlanToJointGoal"), BT::NodeStatus::FAILURE);
  ASSERT_EQ(planHome("PlanToJointGoalFromBoundedStart"), BT::NodeStatus::SUCCESS);
  const auto plan = tree.rootBlackboard()->get<JointTrajectory>("plan");
  EXPECT_DOUBLE_EQ(column(plan, "joint3").front(), -0.01);
  EXPECT_NEAR(column(plan, "joint4").front(), 0.5, 1e-9);
  for (const auto& joint : kArm)
    EXPECT_NEAR(column(plan, joint).back(), 0.0, 1e-6) << joint;
}

TEST_F(HomeRecovery, NativePlannerStartsAShallowSagPastTheBoundWhereTheBoundedStartBeginsAtIt)
{
  if (std::getenv("MOVEIT_LICENSE_KEY") == nullptr)
    GTEST_SKIP() << "Native planning needs a MoveIt Pro license in MOVEIT_LICENSE_KEY.";
  useNativePlanners();
  auto start = kDeepSag;
  start["joint3"] = -0.02;
  measured = withJaw(start, 2.0);
  ASSERT_EQ(planHome("PlanToJointGoal"), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(column(tree.rootBlackboard()->get<JointTrajectory>("plan"), "joint3").front(), -0.02);
  ASSERT_EQ(planHome("PlanToJointGoalFromBoundedStart"), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(column(tree.rootBlackboard()->get<JointTrajectory>("plan"), "joint3").front(), -0.01);
}

TEST_F(HomeRecovery, LeaderHomeReturnPlansBothLegsFromTheBoundedArmWhenAJointSagsPastItsLimit)
{
  if (std::getenv("MOVEIT_LICENSE_KEY") == nullptr)
    GTEST_SKIP() << "Native planning needs a MoveIt Pro license in MOVEIT_LICENSE_KEY.";
  useNativePlanners();
  measured = withJaw(kDeepSag, 2.0);
  EXPECT_EQ(run(kReturn), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(executed.size(), 2U);
  EXPECT_DOUBLE_EQ(column(executed[0], "joint3").front(), -0.01);
  EXPECT_DOUBLE_EQ(column(executed[1], "joint3").front(), -0.01);
  for (const auto& position : column(executed[0], "joint4"))
    EXPECT_NEAR(position, 0.5, 1e-6);
  EXPECT_NEAR(column(executed[0], "joint5").back(), 0.0, 1e-6);
  for (const auto& trajectory : executed)
    for (const auto& position : column(trajectory, "joint7"))
      EXPECT_DOUBLE_EQ(position, 2.0);
  for (const auto& [joint, position] : kHomePose)
    EXPECT_NEAR(measured.at(joint), position, 1e-6) << joint;
  EXPECT_DOUBLE_EQ(measured.at("joint7"), 0.0);
  EXPECT_EQ(home_tail, (std::vector<std::string>{ "settle", "verify", "close" }));
}

TEST_F(HomeRecovery, HomeLegHoldsAStaleJawWhereItMeasures)
{
  measured = withJaw(kSaggedHome, 2.0);
  EXPECT_EQ(run(kHome, withJaw(kHomePose, 5.25)), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(executed.size(), 1U);
  for (const auto& position : column(executed.front(), "joint7"))
    EXPECT_DOUBLE_EQ(position, 2.0);
  EXPECT_DOUBLE_EQ(committed().at("joint7"), 2.0);
}

TEST_F(HomeRecovery, HomeLegKeepsTheGraspSqueeze)
{
  measured = withJaw(kSaggedHome, 2.8615);
  EXPECT_EQ(run(kHome, withJaw(kHomePose, 2.8125)), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(executed.size(), 1U);
  for (const auto& position : column(executed.front(), "joint7"))
    EXPECT_DOUBLE_EQ(position, 2.8125);
  EXPECT_DOUBLE_EQ(committed().at("joint7"), 2.8125);
}

TEST_F(HomeRecovery, EntireInclusiveHomeBoxSkipsArmMotionAndClosesJaw)
{
  for (const auto& joint : kArm)
    for (const double edge : { -0.08, 0.08 })
    {
      measured = withJaw(kHomePose, 2.0);
      measured[joint] = edge;
      EXPECT_EQ(run(kReturn), BT::NodeStatus::SUCCESS) << joint << " " << edge;
      EXPECT_TRUE(executed.empty());
      EXPECT_EQ(plans, 0);
      EXPECT_EQ(fallbacks, 0);
    }
  EXPECT_EQ(switches, 24);  // The wrapper's switch, then Under Power's own.
  EXPECT_EQ(spring_restores.size(), 12U);
  EXPECT_EQ(std::ranges::count(home_tail, "close"), 12);
  EXPECT_DOUBLE_EQ(measured.at("joint7"), 0.0);
}

TEST_F(HomeRecovery, FailedEngageNeverSwitchesPlansExecutesOrCloses)
{
  measured = withJaw(kDeepSag, 2.0);
  engage_refuses = true;
  EXPECT_EQ(run(kReturn), BT::NodeStatus::FAILURE);
  EXPECT_EQ(engages, 1);
  EXPECT_EQ(switches, 0);
  EXPECT_TRUE(spring_restores.empty());
  EXPECT_EQ(plans, 0);
  EXPECT_TRUE(executed.empty());
  EXPECT_TRUE(home_tail.empty());
}

TEST_F(HomeRecovery, ThermalUnderPowerRecoveryDoesNotTryToEngage)
{
  measured = withJaw(kHomePose, 2.0);
  engage_refuses = true;
  EXPECT_EQ(run(R"(<SubTree ID="Return reBot Leader Home Under Power"/>)"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(engages, 0);
  EXPECT_EQ(switches, 1);
  EXPECT_TRUE(spring_restores.empty());  // The guard restores its own gains.
}

TEST_F(HomeRecovery, ReturnEndsFloatBeforeHomeMotion)
{
  // A floating arm (kp=0) is re-anchored and stiffened before HOME plans.
  measured = withJaw(kDeepSag, 2.0);
  EXPECT_EQ(run(kReturn), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(spring_restores.size(), 1U);
  EXPECT_EQ(spring_restores.front(), std::make_tuple(1, 1, std::size_t{ 0 }));
  EXPECT_EQ(executed.size(), 2U);
  EXPECT_EQ(measured, withJaw(kHomePose, 0.0));
}

TEST_F(HomeRecovery, AtHomeSettlesVerifiesThenClosesAtNominalZero)
{
  measured = withJaw(kHomePose, 2.0);
  ASSERT_EQ(run(kReturn), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(executed.empty());
  EXPECT_EQ(home_tail, (std::vector<std::string>{ "settle", "verify", "close" }));
  EXPECT_DOUBLE_EQ(measured.at("joint7"), 0.0);
}

TEST_F(HomeRecovery, NoLeaderNotificationDoesNotCallLeaderAndKeepsHomeThenJaw)
{
  useNoLeaderNotification();
  measured = withJaw(kDeepSag, 2.0);
  EXPECT_EQ(run(kReturn), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(executed.size(), 2U);
  EXPECT_EQ(measured, withJaw(kHomePose, 0.0));
  EXPECT_EQ(home_tail, (std::vector<std::string>{ "settle", "close" }));
}

TEST_F(HomeRecovery, NoLeaderNotificationDoesNotSwallowHomeFailure)
{
  useNoLeaderNotification();
  measured = withJaw(kDeepSag, 2.0);
  execution_refuses = true;
  EXPECT_EQ(run(kReturn), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(home_tail.empty());
  EXPECT_EQ(measured, withJaw(kDeepSag, 2.0));
}

TEST_F(HomeRecovery, RelaxWithoutLeaderKeepsArmHomeJawDwellThenGuardedRelax)
{
  useNoLeaderNotification();
  measured = withJaw(kDeepSag, 2.0);
  EXPECT_EQ(run(R"(<SubTree ID="Relax at HOME"/>)"), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(executed.size(), 2U);
  EXPECT_EQ(measured, withJaw(kHomePose, 0.0));
  EXPECT_EQ(home_tail, (std::vector<std::string>{ "settle", "close", "dwell", "relax" }));
  EXPECT_EQ(idle_checks, 0);  // Stock no-leader idle leaf calls no service.
}

TEST_F(HomeRecovery, RelaxEndsFloatBeforeHomeMotion)
{
  // A floating arm sags out of the HOME box; spring gains return before HOME plans.
  useNoLeaderNotification();
  measured = withJaw(kDeepSag, 2.0);
  EXPECT_EQ(run(R"(<SubTree ID="Relax at HOME"/>)"), BT::NodeStatus::SUCCESS);
  // Relax's own restore, then the HOME wrapper's; both before any motion.
  ASSERT_EQ(spring_restores.size(), 2U);
  EXPECT_EQ(spring_restores.front(), std::make_tuple(1, 1, std::size_t{ 0 }));
  EXPECT_EQ(spring_restores.back(), std::make_tuple(2, 2, std::size_t{ 0 }));
  EXPECT_EQ(engages, 2);  // HOME's own engage is the idempotent already-ON path.
  EXPECT_EQ(measured, withJaw(kHomePose, 0.0));
  EXPECT_EQ(home_tail, (std::vector<std::string>{ "settle", "close", "dwell", "relax" }));
}

TEST_F(HomeRecovery, RelaxWithoutLeaderHomeFailureNeverClosesDwellsOrRelaxes)
{
  useNoLeaderNotification();
  measured = withJaw(kDeepSag, 2.0);
  execution_refuses = true;
  EXPECT_EQ(run(R"(<SubTree ID="Relax at HOME"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(home_tail.empty());
  EXPECT_EQ(measured, withJaw(kDeepSag, 2.0));
  EXPECT_EQ(idle_checks, 0);
}

TEST_F(HomeRecovery, RelaxWithoutLeaderDoesNotSwallowRelaxRefusal)
{
  useNoLeaderNotification();
  measured = withJaw(kHomePose, 2.0);
  relax_refuses = true;
  EXPECT_EQ(run(R"(<SubTree ID="Relax at HOME"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(home_tail, (std::vector<std::string>{ "settle", "close", "dwell", "relax" }));
  EXPECT_EQ(idle_checks, 0);
}

TEST_F(HomeRecovery, RelaxWithLeaderStillNotifiesAndChecksIdleTwice)
{
  factory.registerBehaviorTreeFromFile(BOSTON_SHARE "/objectives/rebot_check_leader_idle.xml");
  measured = withJaw(kHomePose, 2.0);
  EXPECT_EQ(run(R"(<SubTree ID="Relax at HOME"/>)"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(home_tail, (std::vector<std::string>{ "settle", "verify", "close", "dwell", "relax" }));
  EXPECT_EQ(idle_checks, 2);
}

TEST_F(HomeRecovery, RefusedOptionalNotificationCannotSwallowGuardedRelaxRefusal)
{
  factory.registerBehaviorTreeFromFile(BOSTON_SHARE "/objectives/rebot_check_leader_idle.xml");
  measured = withJaw(kHomePose, 2.0);
  verification_refuses = true;
  relax_refuses = true;
  EXPECT_EQ(run(R"(<SubTree ID="Relax at HOME"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(home_tail, (std::vector<std::string>{ "settle", "verify", "close", "dwell", "relax" }));
  EXPECT_EQ(idle_checks, 2);
}

TEST_F(HomeRecovery, LeaderIdleRefusalNeverEngagesMovesNotifiesOrRelaxes)
{
  factory.registerBehaviorTreeFromFile(BOSTON_SHARE "/objectives/rebot_check_leader_idle.xml");
  measured = withJaw(kDeepSag, 2.0);
  idle_refuses = true;
  EXPECT_EQ(run(R"(<SubTree ID="Relax at HOME"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(idle_checks, 1);
  EXPECT_EQ(engages, 0);
  EXPECT_TRUE(spring_restores.empty());
  EXPECT_TRUE(executed.empty());
  EXPECT_TRUE(home_tail.empty());
}

TEST_F(HomeRecovery, MissingOrRejectedVerificationStillClosesAfterHome)
{
  measured = withJaw(kHomePose, 2.0);
  verification_refuses = true;
  EXPECT_EQ(run(kReturn), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(home_tail, (std::vector<std::string>{ "settle", "verify", "close" }));
  EXPECT_DOUBLE_EQ(measured.at("joint7"), 0.0);
}

TEST_F(HomeRecovery, FailedCloseDoesNotTurnSuccessfulArmHomeIntoFailure)
{
  measured = withJaw(kHomePose, 2.0);
  close_refuses = true;
  EXPECT_EQ(run(kReturn), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(home_tail, (std::vector<std::string>{ "settle", "verify", "close" }));
  EXPECT_DOUBLE_EQ(measured.at("joint7"), 2.0);
}

TEST_F(HomeRecovery, YawFirstThenRestFromBoundedStartAtSourceSpeedWhileHoldingJaw)
{
  measured = withJaw(kDeepSag, 2.0);
  ASSERT_EQ(run(kReturn), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(executed.size(), 2U);
  EXPECT_DOUBLE_EQ(column(executed[0], "joint3").front(), -0.01);
  EXPECT_DOUBLE_EQ(column(executed[1], "joint3").front(), -0.01);
  for (const auto& position : column(executed[0], "joint4"))
    EXPECT_DOUBLE_EQ(position, 0.5);
  EXPECT_NEAR(column(executed[0], "joint5").back(), 0.0, 1e-12);
  for (const auto& trajectory : executed)
  {
    for (const auto& point : trajectory.points)
      for (const auto velocity : point.velocities)
        EXPECT_LE(std::abs(velocity), 0.05 + 1e-9);
    for (std::size_t i = 1; i < trajectory.points.size(); ++i)
      for (std::size_t joint = 0; joint < trajectory.joint_names.size(); ++joint)
        EXPECT_LE(
            std::abs(trajectory.points[i].positions[joint] - trajectory.points[i - 1].positions[joint]) /
                (seconds(trajectory.points[i].time_from_start) - seconds(trajectory.points[i - 1].time_from_start)),
            0.05 + 1e-9);
    for (const auto& position : column(trajectory, "joint7"))
      EXPECT_DOUBLE_EQ(position, 2.0);
  }
  EXPECT_EQ(measured, withJaw(kHomePose, 0.0));
  EXPECT_EQ(home_tail, (std::vector<std::string>{ "settle", "verify", "close" }));
  EXPECT_EQ(fallbacks, 0);
  EXPECT_EQ(switches, 2);  // The wrapper's switch, then Under Power's own.
}

TEST_F(HomeRecovery, RestHasCollisionCheckedFallbackButYawDoesNot)
{
  measured = withJaw(kDeepSag, 2.0);
  planner_refuses_start = true;
  EXPECT_EQ(run(kReturn), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(executed.empty());
  EXPECT_TRUE(home_tail.empty());
  EXPECT_EQ(fallbacks, 0);
  measured["joint5"] = 0.0;
  ASSERT_EQ(run(kReturn), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fallbacks, 1);
  EXPECT_EQ(validations, 2);
  ASSERT_EQ(executed.size(), 1U);
  EXPECT_DOUBLE_EQ(column(executed[0], "joint3").front(), -0.01);
  EXPECT_EQ(measured, withJaw(kHomePose, 0.0));
  EXPECT_EQ(home_tail, (std::vector<std::string>{ "settle", "verify", "close" }));
}

TEST_F(HomeRecovery, RefusedAnchoredFallbackDoesNotExecuteOrCommit)
{
  measured = withJaw(kSaggedHome, 2.0);
  measured["joint4"] = 0.5;
  planner_refuses_start = true;
  refused_validation = 2;
  const auto command = withJaw(kHomePose, 5.25);
  EXPECT_EQ(run(kHome, command), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(executed.empty());
  EXPECT_EQ(validations, 2);
  EXPECT_EQ(committed(), command);
}

TEST_F(HomeRecovery, FailedExecutionDoesNotCommitCandidateEndpoint)
{
  measured = withJaw(kSaggedHome, 2.0);
  measured["joint4"] = 0.5;
  execution_refuses = true;
  const auto command = measured;
  EXPECT_EQ(run(kHome, command), BT::NodeStatus::FAILURE);
  ASSERT_EQ(executed.size(), 1U);
  EXPECT_EQ(committed(), command);
  EXPECT_EQ(measured, command);
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
