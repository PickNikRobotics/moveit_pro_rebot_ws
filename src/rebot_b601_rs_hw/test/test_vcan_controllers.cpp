// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
// Production configuration, ControllerManager, controllers, driver and SDK.
// Only the seven motors and planning-scene response are simulated; vcan only.

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <control_msgs/msg/joint_jog.hpp>
#include <controller_manager/controller_manager.hpp>
#include <fstream>
#include <future>
#include <iterator>
#include <joint_trajectory_admittance_controller/joint_trajectory_admittance_controller.hpp>
#include <joint_velocity_controller/joint_velocity_controller.hpp>
#include <memory>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_pro_controllers_msgs/action/follow_joint_trajectory_with_admittance.hpp>
#include <moveit_pro_controllers_msgs/msg/velocity_force_command.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robstride_interfaces/srv/set_data_to_robstride.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <thread>
#include <vector>

#include "fake_robstride_motor.hpp"
#include "vcan_controller_fixture.hpp"

namespace
{
using namespace std::chrono_literals;
using JTAC = moveit_pro_controllers::JointTrajectoryAdmittanceController;
using Jog = moveit_pro_controllers::JointVelocityController;
using rebot_vcan_test::Configured;
using Action = moveit_pro_controllers_msgs::action::FollowJointTrajectoryWithAdmittance;
using Values = std::array<double, 7>;
const std::string trajectory_name = "joint_trajectory_admittance_controller";
const std::string arm_jog_name = "joint_velocity_controller";
const std::string jaw_jog_name = "gripper_joint_velocity_controller";
const std::string pose_jog_name = "velocity_force_controller";
// Public joint contract; JointVelocityController keeps its own joint list.
const std::vector<std::string> arm_joints{ "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
const std::vector<std::string> jaw_joints{ "joint7" };

void Require(bool condition, const std::string& reason)
{
  if (!condition)
    throw std::runtime_error(reason);
}

std::string ReadFile(const std::string& path)
{
  std::ifstream file(path);
  Require(file.good(), "cannot read " + path);
  return { std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>() };
}

struct Manager : controller_manager::ControllerManager
{
  using ControllerManager::ControllerManager;
  auto& resources()
  {
    return *resource_manager_;
  }
};

class VcanControllers : public testing::Test
{
protected:
  void SetUp() override
  {
    if (std::string(robstride_test::TestInterface()) != "vcan0" ||
        !robstride_test::IsVirtualCan(robstride_test::TestInterface()))
    {
      if (std::getenv("ROBSTRIDE_REQUIRE_VCAN"))
        FAIL() << "qualification requires kernel vcan0, never a physical device";
      GTEST_SKIP() << "run driver test/with_vcan.py in an isolated network namespace";
    }
    follower = std::make_unique<robstride_test::FakeRobstrideFollower>("vcan0");
    ASSERT_TRUE(follower->ok());
    SetPose({ .2, .65, .85, -.2, .2, -.3, .8 });
    urdf = ReadFile(TEST_VCAN_URDF);
    const auto mock_share = ament_index_cpp::get_package_share_directory("rebot_b601_rs_mock");
    const auto yaml = mock_share + "/config/control/rebot.ros2_control.yaml";
    srdf = ReadFile(mock_share + "/config/moveit/rebot.srdf");
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    manager = std::make_shared<Manager>(executor, urdf, true, "vcan_controller_manager");
    executor->add_node(manager);
    ASSERT_TRUE(manager->is_resource_manager_initialized());
    node = std::make_shared<rclcpp::Node>("vcan_controller_observer");
    executor->add_node(node);
    trajectory = std::make_shared<Configured<JTAC>>(yaml);
    arm_jog = std::make_shared<Configured<Jog>>(yaml);
    jaw_jog = std::make_shared<Configured<Jog>>(yaml);
    pose_jog = rebot_vcan_test::MakePoseJog(yaml);
    Add(trajectory, *trajectory, trajectory_name,
        "joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController");
    Add(arm_jog, *arm_jog, arm_jog_name, "joint_velocity_controller/JointVelocityController");
    Add(jaw_jog, *jaw_jog, jaw_jog_name, "joint_velocity_controller/JointVelocityController");
    Add(pose_jog.controller, *pose_jog.model, pose_jog_name, "velocity_force_controller/VelocityForceController");
    auto broadcaster =
        manager->load_controller("joint_state_broadcaster", "joint_state_broadcaster/JointStateBroadcaster");
    ASSERT_NE(broadcaster, nullptr);
    ASSERT_EQ(manager->configure_controller("joint_state_broadcaster"), controller_interface::return_type::OK);
    auto gravity = manager->load_controller("gravity_compensation_controller",
                                            "kdl_gravity_compensation_controller/GravityCompensationController");
    ASSERT_NE(gravity, nullptr);
    const auto settings = YAML::LoadFile(yaml)["gravity_compensation_controller"]["ros__parameters"];
    ASSERT_TRUE(gravity->get_node()
                    ->set_parameters_atomically(
                        { rclcpp::Parameter("joints", settings["joints"].as<std::vector<std::string>>()),
                          rclcpp::Parameter("root_link", settings["root_link"].as<std::string>()),
                          rclcpp::Parameter("tip_link", settings["tip_link"].as<std::string>()),
                          rclcpp::Parameter("gravity_vector", settings["gravity_vector"].as<std::vector<double>>()),
                          rclcpp::Parameter("payload_mass", settings["payload_mass"].as<double>()),
                          rclcpp::Parameter("payload_com", settings["payload_com"].as<std::vector<double>>()),
                          rclcpp::Parameter("gains", settings["gains"].as<std::vector<double>>()),
                          rclcpp::Parameter("offsets", settings["offsets"].as<std::vector<double>>()) })
                    .successful);
    ASSERT_EQ(manager->configure_controller("gravity_compensation_controller"), controller_interface::return_type::OK);
    state_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message) { joint_states.push_back(*message); });
    arm_publisher = node->create_publisher<control_msgs::msg::JointJog>("/" + arm_jog_name + "/command", 1);
    jaw_publisher = node->create_publisher<control_msgs::msg::JointJog>("/" + jaw_jog_name + "/command", 1);
    pose_publisher = node->create_publisher<moveit_pro_controllers_msgs::msg::VelocityForceCommand>(
        "/" + pose_jog_name + "/command", 1);
    action = rclcpp_action::create_client<Action>(node, "/" + trajectory_name + "/follow_joint_trajectory");
    Switch({ "joint_state_broadcaster", "gravity_compensation_controller", trajectory_name }, {});
    Cycle(100);
    // Planning-group joint names resolve on activation.
    ASSERT_EQ(trajectory->getJointNames(),
              (std::vector<std::string>{ "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7" }));
  }

  void TearDown() override
  {
    // Every switch worker is joined by Switch() before loans are released.
    if (manager)
    {
      EXPECT_TRUE(manager->shutdown_controllers());
      EXPECT_TRUE(manager->resources().shutdown_components());
      for (size_t i = 0; i < 7; ++i)
        EXPECT_FALSE(manager->resources().command_interface_is_claimed("joint" + std::to_string(i + 1) + "/position"));
      executor->remove_node(manager);
    }
    if (executor && node)
      executor->remove_node(node);
    action.reset();
    trajectory.reset();
    arm_jog.reset();
    jaw_jog.reset();
    pose_jog = {};
    manager.reset();
    executor.reset();
    node.reset();
    follower.reset();
  }

  void Add(const controller_interface::ControllerInterfaceBaseSharedPtr& controller,
           moveit_pro_controllers::ControllerWithRobotModel& with_model, const std::string& name,
           const std::string& type)
  {
    Require(manager->add_controller(controller, name, type) != nullptr, "cannot add " + name);
    auto model = std::make_shared<std_msgs::msg::String>();
    model->data = urdf;
    auto semantic = std::make_shared<std_msgs::msg::String>();
    semantic->data = srdf;
    with_model.setRobotDescription(model);
    with_model.setRobotDescriptionSemantic(semantic);
    Require(manager->configure_controller(name) == controller_interface::return_type::OK, "cannot configure " + name);
  }

  Values Measured()
  {
    Values values{};
    for (size_t i = 0; i < 7; ++i)
    {
      auto state = manager->resources().claim_state_interface("joint" + std::to_string(i + 1) + "/position");
      values[i] = state.get_optional().value();
    }
    return values;
  }
  void SetPose(const Values& pose)
  {
    for (size_t i = 0; i < 7; ++i)
      follower->motor(i).set_position(pose[i]);
  }
  void Cycle(int milliseconds = 10, bool callbacks = true)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    do
    {
      const auto time = manager->get_trigger_clock()->now();
      manager->read(time, period);
      Require(manager->update(manager->now(), period) == controller_interface::return_type::OK, "controller update");
      manager->write(time, period);
      if (callbacks)
        executor->spin_some();
      std::this_thread::sleep_for(10ms);
    } while (std::chrono::steady_clock::now() < deadline);
  }

  void Switch(const std::vector<std::string>& start, const std::vector<std::string>& stop)
  {
    auto worker = std::async(std::launch::async, [&]() {
      return manager->switch_controller(start, stop, 2, false, rclcpp::Duration::from_seconds(3));
    });
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (worker.wait_for(0s) != std::future_status::ready && std::chrono::steady_clock::now() < deadline)
      Cycle(10, false);  // CM's list-lock must not be reentered by executor callbacks.
    if (worker.wait_for(0s) != std::future_status::ready)
    {
      std::cerr << "controller switch did not quiesce; failing isolated test process\n";
      std::_Exit(2);
    }
    Require(worker.get() == controller_interface::return_type::OK, "controller switch failed");
    Cycle();
  }

  template <class Future>
  void Await(Future& future, double seconds = 3)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (future.wait_for(0s) != std::future_status::ready && std::chrono::steady_clock::now() < deadline)
      Cycle();
    Require(future.wait_for(0s) == std::future_status::ready, "ROS operation timeout");
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>> Goal(const Values& target, bool stationary = false)
  {
    Require(action->wait_for_action_server(2s), "trajectory server unavailable");
    Action::Goal goal;
    goal.trajectory.header.stamp = node->now();  // Nonzero stamp forces measured start.
    goal.trajectory.joint_names = trajectory->getJointNames();
    trajectory_msgs::msg::JointTrajectoryPoint first, last;
    const auto measured = Measured();
    first.positions.assign(stationary ? target.begin() : measured.begin(), stationary ? target.end() : measured.end());
    first.velocities.assign(7, 0.);
    first.accelerations.assign(7, 0.);
    last.positions.assign(target.begin(), target.end());
    last.velocities.assign(7, 0.);
    last.accelerations.assign(7, 0.);
    last.time_from_start.sec = stationary ? 1 : 2;
    goal.trajectory.points = { first, last };
    auto future = action->async_send_goal(goal);
    Await(future);
    auto handle = future.get();
    Require(handle != nullptr, "trajectory goal rejected");
    return handle;
  }

  void Complete(const std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>>& handle)
  {
    auto future = action->async_get_result(handle);
    Await(future, 5);
    const auto result = future.get();
    Require(result.code == rclcpp_action::ResultCode::SUCCEEDED, "trajectory did not succeed");
  }

  void Gain(size_t id, const std::string& gain, float value)
  {
    auto client = node->create_client<robstride_interfaces::srv::SetDataToRobstride>(
        "/robstride_hardware_interface/set_data_to_robstride");
    Require(client->wait_for_service(2s), "gain service missing");
    auto request = std::make_shared<robstride_interfaces::srv::SetDataToRobstride::Request>();
    request->id = id;
    request->item_name = gain;
    request->item_data = value;
    auto future = client->async_send_request(request);
    Await(future);
    Require(future.get()->result, "gain not queued");
  }

  void JogFor(bool jaw, double velocity, int milliseconds)
  {
    auto publisher = jaw ? jaw_publisher : arm_publisher;
    const auto ready_deadline = std::chrono::steady_clock::now() + 2s;
    while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < ready_deadline)
      Cycle();
    Require(publisher->get_subscription_count() > 0, "jog commands subscriber unavailable");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    do
    {
      control_msgs::msg::JointJog command;
      command.header.stamp = node->now();
      command.joint_names = jaw ? jaw_joints : arm_joints;
      command.velocities.assign(command.joint_names.size(), 0.);
      command.velocities[0] = velocity;
      publisher->publish(command);
      Cycle();
    } while (std::chrono::steady_clock::now() < deadline);
  }

  std::string urdf, srdf;
  std::unique_ptr<robstride_test::FakeRobstrideFollower> follower;
  std::shared_ptr<Manager> manager;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Configured<JTAC>> trajectory;
  std::shared_ptr<Configured<Jog>> arm_jog, jaw_jog;
  rebot_vcan_test::PoseJog pose_jog;
  rclcpp_action::Client<Action>::SharedPtr action;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr arm_publisher, jaw_publisher;
  rclcpp::Publisher<moveit_pro_controllers_msgs::msg::VelocityForceCommand>::SharedPtr pose_publisher;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription;
  std::vector<sensor_msgs::msg::JointState> joint_states;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(.01);
};

TEST_F(VcanControllers, TrajectoryDrivesSevenPublicMotorCoordinatesAndRecordsFreshState)
{
  const Values target{ .3, .75, .95, -.1, .1, -.2, 1.1 };
  auto handle = Goal(target);
  Complete(handle);
  Cycle(100);
  const auto measured = Measured();
  double worst = 0;
  for (size_t i = 0; i < 7; ++i)
  {
    EXPECT_NEAR(measured[i], target[i], .003);
    EXPECT_NEAR(follower->motor(i).position(), target[i], .003);
    worst = std::max({ worst, std::abs(measured[i] - target[i]), std::abs(follower->motor(i).position() - target[i]) });
  }
  ASSERT_GE(joint_states.size(), 50u);
  EXPECT_LT(rclcpp::Time(joint_states.front().header.stamp), rclcpp::Time(joint_states.back().header.stamp));
  std::cout << "trajectory: " << joint_states.size() << " recorded JointState samples; final max |error| " << worst
            << " rad (limit 0.003)\n";
}

TEST_F(VcanControllers, ArmJogSignsStopOnTimeoutAndLeaveJawUntouched)
{
  Switch({ arm_jog_name }, { trajectory_name });
  for (size_t i = 0; i < 7; ++i)
    EXPECT_EQ(manager->resources().command_interface_is_claimed("joint" + std::to_string(i + 1) + "/position"), i < 6);
  const auto initial = Measured();
  JogFor(false, .1, 500);
  Cycle(600);  // Public command_timeout=0.2 plus bounded deceleration.
  const auto positive = Measured();
  EXPECT_GT(positive[0] - initial[0], .02);
  EXPECT_NEAR(positive[6], initial[6], .003);
  const double stopped = positive[0];
  Cycle(250);
  EXPECT_NEAR(Measured()[0], stopped, .003);
  JogFor(false, -.1, 500);
  Cycle(600);
  EXPECT_LT(Measured()[0] - positive[0], -.02);
  for (size_t i = 1; i < 7; ++i)
    EXPECT_NEAR(Measured()[i], initial[i], .003);
}

TEST_F(VcanControllers, GripperJogUsesRadiansAndNativeJawMetresWithoutClaimingMimics)
{
  // Arm and jaw jog have disjoint position claims and can run together.
  Switch({ arm_jog_name, jaw_jog_name }, { trajectory_name });
  for (size_t i = 0; i < 7; ++i)
    EXPECT_TRUE(manager->resources().command_interface_is_claimed("joint" + std::to_string(i + 1) + "/position"));
  const auto initial = Measured();
  JogFor(true, .5, 500);
  Cycle(600);
  const auto positive = Measured();
  EXPECT_GT(positive[6] - initial[6], .1);
  for (size_t i = 0; i < 6; ++i)
    EXPECT_NEAR(positive[i], initial[i], .003);
  auto& model_state = jaw_jog->getRobotState();
  model_state.setVariablePosition("joint7", positive[6]);
  model_state.update();
  EXPECT_NEAR(model_state.getVariablePosition("gripper_joint1"), .008 * positive[6], 1e-12);
  EXPECT_NEAR(model_state.getVariablePosition("gripper_joint2"), .008 * positive[6], 1e-12);
  EXPECT_FALSE(manager->resources().command_interface_exists("gripper_joint1/position"));
  EXPECT_FALSE(manager->resources().command_interface_exists("gripper_joint2/position"));
  JogFor(true, -.5, 500);
  Cycle(600);
  EXPECT_LT(Measured()[6] - positive[6], -.1);
}

TEST_F(VcanControllers, PoseJogDrivesCartesianSignAndStopsOnTimeout)
{
  Switch({ pose_jog_name, jaw_jog_name }, { trajectory_name });
  auto tool = [&]() {
    auto& model = pose_jog.model->getRobotState();
    const auto measured = Measured();
    for (size_t i = 0; i < 7; ++i)
      model.setVariablePosition("joint" + std::to_string(i + 1), measured[i]);
    model.update();
    return Eigen::Vector3d(model.getGlobalLinkTransform("gripper_end").translation());
  };
  auto stream = [&](double velocity) {
    const auto ready = std::chrono::steady_clock::now() + 2s;
    while (pose_publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < ready)
      Cycle();
    Require(pose_publisher->get_subscription_count() > 0, "pose jog subscriber missing");
    const auto deadline = std::chrono::steady_clock::now() + 700ms;
    while (std::chrono::steady_clock::now() < deadline)
    {
      moveit_pro_controllers_msgs::msg::VelocityForceCommand message;
      message.header.stamp = node->now();
      message.control_frame_id = "base_link";
      message.velocity_controlled_axes.x = true;
      message.twist.linear.x = velocity;
      pose_publisher->publish(message);
      Cycle();
    }
    Cycle(700);
  };
  const double jaw = Measured()[6];
  const auto initial = tool();
  stream(.01);
  const auto positive = tool();
  EXPECT_GT(positive.x() - initial.x(), .003);
  Cycle(300);
  EXPECT_LT((tool() - positive).norm(), .002);
  stream(-.01);
  EXPECT_LT(tool().x() - positive.x(), -.003);
  EXPECT_NEAR(Measured()[6], jaw, .003);
}

TEST_F(VcanControllers, ReturningFromJogUsesStampedMeasuredPoseAnchor)
{
  Switch({ arm_jog_name, jaw_jog_name }, { trajectory_name });
  JogFor(false, .1, 400);
  JogFor(true, .5, 400);
  Cycle(600);
  const auto measured = Measured();
  // Float all arm joints while switching: old JTAC command may be retained.
  for (size_t i = 1; i <= 6; ++i)
    Gain(i, "kp", 0);
  Cycle(100);
  const auto start = robstride_test::SteadyNs();
  Switch({ trajectory_name }, { arm_jog_name, jaw_jog_name });
  auto anchor = Goal(measured, true);
  Complete(anchor);
  Cycle(100);
  const auto command = trajectory->getStateReference().positions;
  ASSERT_EQ(command.size(), 7);
  for (size_t i = 0; i < 7; ++i)
    EXPECT_NEAR(command[i], measured[i], .003);
  // J7 stays stiff, so a stale pre-jog anchor at activation would snap it.
  size_t stiff_frames = 0;
  for (const auto& sample : follower->motor(6).samples())
    if (sample.stamp_ns >= start && sample.type == 1 && sample.kp > 0)
    {
      ++stiff_frames;
      EXPECT_NEAR(sample.target, sample.before, .003);
    }
  EXPECT_GT(stiff_frames, 0u);
}

TEST_F(VcanControllers, RestoringStiffnessRunsSourceObjectiveAfterMeasuredReanchorNoSnap)
{
  // Float at a held trajectory, then inject an independently measured pose.
  // The fake does not chase Type-1 targets when kp is zero.
  for (size_t i = 1; i <= 6; ++i)
    Gain(i, "kp", 0);
  Cycle(100);
  const Values floated{ -.2, 1.2, 1.3, .4, -.3, .4, .8 };
  SetPose(floated);
  Cycle(100);
  auto commands = trajectory->getStateReference().positions;
  ASSERT_EQ(commands.size(), 7);
  EXPECT_GT(std::abs(commands[1] - Measured()[1]), .3) << "fixture needs a genuinely stale controller target";
  auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false);
  context->robot_model =
      std::const_pointer_cast<moveit_pro::base::RobotModel>(trajectory->getRobotState().getRobotModel());
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
  BT::BehaviorTreeFactory factory;
  std::vector<std::shared_ptr<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>> plugins;
  for (const auto* name :
       { "moveit_pro::behaviors::CoreBehaviorsLoader", "moveit_pro::behaviors::MTCCoreBehaviorsLoader",
         "moveit_pro::behaviors::VisionBehaviorsLoader", "moveit_pro::behaviors::ConverterBehaviorsLoader",
         "rebot_b601_rs_hw::RebotHardwareBehaviorsLoader" })
  {
    auto plugin = loader.createSharedInstance(name);
    plugin->registerBehaviors(factory, context);
    plugins.push_back(plugin);
  }
  const auto joints = trajectory->getJointNames();
  auto scene = node->create_service<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene", [&](moveit_msgs::srv::GetPlanningScene::Request::SharedPtr,
                                 moveit_msgs::srv::GetPlanningScene::Response::SharedPtr response) {
        const auto measured = Measured();
        response->scene.robot_state.joint_state.name = joints;
        response->scene.robot_state.joint_state.position.assign(measured.begin(), measured.end());
        response->scene.robot_state.joint_state.velocity.assign(7, 0.);
        response->scene.robot_state.joint_state.header.stamp = node->now();
      });
  const auto start = robstride_test::SteadyNs();
  const auto share = ament_index_cpp::get_package_share_directory("rebot_b601_rs_hw");
  auto tree = factory.createTreeFromFile(share + "/objectives/stop_gravity_comp.xml");
  auto status = BT::NodeStatus::RUNNING;
  const auto deadline = std::chrono::steady_clock::now() + 12s;
  while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
  {
    status = tree.tickOnce();
    Cycle();
  }
  if (status == BT::NodeStatus::RUNNING)
    tree.haltTree();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS) << context->logger->consumeErrorLogBuffer();
  Cycle(100);
  double worst = 0;
  size_t stiff_frames = 0;
  for (size_t i = 0; i < 6; ++i)
  {
    bool saw_restoration = false;
    for (const auto& sample : follower->motor(i).samples())
      if (sample.stamp_ns >= start && sample.type == 1 && sample.kp > 0)
      {
        saw_restoration = true;
        ++stiff_frames;
        worst = std::max(worst, std::abs(sample.target - sample.before));
        // Compare against BEFORE the ideal follower moves. Comparing only
        // position after the command would hide a stiffness-induced snap.
        EXPECT_NEAR(sample.target, sample.before, .003);
      }
    EXPECT_TRUE(saw_restoration);
    EXPECT_NEAR(follower->motor(i).position(), floated[i], .003);
  }
  std::cout << "Stop Gravity Comp: " << stiff_frames
            << " stiff (kp>0) frames on J1-J6, max |target - pose before frame| " << worst << " rad (limit 0.003)\n";
}

}  // namespace

int main(int argc, char** argv)
{
  std::cout << std::unitbuf;
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
