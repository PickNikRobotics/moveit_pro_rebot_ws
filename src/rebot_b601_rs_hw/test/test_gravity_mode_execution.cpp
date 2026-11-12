// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
// Adapted from the published gravity-mode work's offline ordering proof.
// Actual ControllerManager, JTAC, chainable gravity and source Objective leaves;
// only memory hardware, planning-scene responses and motor gain service are fake.
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <cmath>
#include <controller_manager/controller_manager.hpp>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <future>
#include <iostream>
#include <iterator>
#include <joint_trajectory_admittance_controller/joint_trajectory_admittance_controller.hpp>
#include <kdl_gravity_compensation_controller/gravity_compensation_controller.hpp>
#include <kdl_gravity_compensation_controller/gravity_model.hpp>
#include <memory>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robstride_interfaces/srv/set_data_to_robstride.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using Controller = moveit_pro_controllers::JointTrajectoryAdmittanceController;
using Values = std::array<double, 7>;
using GainService = robstride_interfaces::srv::SetDataToRobstride;
void require(bool condition, const std::string& reason)
{
  if (!condition)
    throw std::runtime_error(reason);
}
std::string readFile(const std::string& path)
{
  std::ifstream file(path);
  require(file.good(), "cannot read " + path);
  return { std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>() };
}
struct ObservedJTAC : Controller
{
  size_t activations = 0, deactivations = 0, updates = 0;
  rclcpp::NodeOptions define_custom_node_options() const override
  {
    auto options = Controller::define_custom_node_options();
    options.parameter_overrides(
        { rclcpp::Parameter("planning_group_name", "manipulator_and_gripper"),
          rclcpp::Parameter("sensor_frames", std::vector<std::string>{ "gripper_end" }),
          rclcpp::Parameter("ee_frames", std::vector<std::string>{ "gripper_end" }),
          rclcpp::Parameter("stop_accelerations", std::vector<double>{ .6, .6, .6, .6, .6, .6, 6. }),
          rclcpp::Parameter("default_path_tolerance", .20), rclcpp::Parameter("default_goal_tolerance", .1),
          rclcpp::Parameter("default_goal_time_tolerance", 2.) });
    return options;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override
  {
    ++activations;
    return Controller::on_activate(state);
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override
  {
    ++deactivations;
    return Controller::on_deactivate(state);
  }
  controller_interface::return_type update(const rclcpp::Time& t, const rclcpp::Duration& p) override
  {
    ++updates;
    return Controller::update(t, p);
  }
};
struct Manager : controller_manager::ControllerManager
{
  using ControllerManager::ControllerManager;
  auto& resources()
  {
    return *resource_manager_;
  }
};
struct StateAccess : hardware_interface::LoanedStateInterface
{
  explicit StateAccess(hardware_interface::LoanedStateInterface&& h) : LoanedStateInterface(std::move(h))
  {
  }
  auto* pointer()
  {
    return &const_cast<hardware_interface::StateInterface&>(state_interface_);
  }
};
struct CommandAccess : hardware_interface::LoanedCommandInterface
{
  explicit CommandAccess(hardware_interface::LoanedCommandInterface&& h) : LoanedCommandInterface(std::move(h))
  {
  }
  auto* pointer()
  {
    return &command_interface_;
  }
};
void runModeTest(const std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>& executor,
                 const std::shared_ptr<Manager>& manager, const std::string& urdf, const std::string& fault)
{
  const auto mock_share = ament_index_cpp::get_package_share_directory("rebot_b601_rs_mock");
  const auto hardware_share = ament_index_cpp::get_package_share_directory("rebot_b601_rs_hw");
  const auto srdf = readFile(mock_share + "/config/moveit/rebot.srdf");
  const auto settings = YAML::LoadFile(
      mock_share + "/config/control/rebot.ros2_control.yaml")["gravity_compensation_controller"]["ros__parameters"];
  require(manager->is_resource_manager_initialized(), "resource manager not initialized");
  const std::string name = "joint_trajectory_admittance_controller";
  auto controller = std::make_shared<ObservedJTAC>();
  require(manager->add_controller(controller, name,
                                  "joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController") !=
              nullptr,
          "add failed");
  auto description = std::make_shared<std_msgs::msg::String>();
  description->data = urdf;
  auto semantic = std::make_shared<std_msgs::msg::String>();
  semantic->data = srdf;
  controller->setRobotDescription(description);
  controller->setRobotDescriptionSemantic(semantic);
  require(manager->configure_controller(name) == controller_interface::return_type::OK, "configure failed");
  const std::string gravity_name = "gravity_compensation_controller";
  auto gravity =
      manager->load_controller(gravity_name, "kdl_gravity_compensation_controller/GravityCompensationController");
  require(gravity != nullptr, "load actual gravity plugin failed");
  require(gravity->get_node()
              ->set_parameters_atomically(
                  { rclcpp::Parameter("joints", settings["joints"].as<std::vector<std::string>>()),
                    rclcpp::Parameter("root_link", settings["root_link"].as<std::string>()),
                    rclcpp::Parameter("tip_link", settings["tip_link"].as<std::string>()),
                    rclcpp::Parameter("gravity_vector", settings["gravity_vector"].as<std::vector<double>>()),
                    rclcpp::Parameter("payload_mass", settings["payload_mass"].as<double>()),
                    rclcpp::Parameter("payload_com", settings["payload_com"].as<std::vector<double>>()),
                    rclcpp::Parameter("gains", settings["gains"].as<std::vector<double>>()),
                    rclcpp::Parameter("offsets", settings["offsets"].as<std::vector<double>>()) })
              .successful,
          "configure gravity parameters failed");
  require(manager->configure_controller(gravity_name) == controller_interface::return_type::OK,
          "configure gravity failed");
  const auto joints = controller->getJointNames();
  require(joints == std::vector<std::string>{ "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7" },
          "joint order");
  std::array<hardware_interface::StateInterface*, 7> state{};
  std::array<hardware_interface::CommandInterface*, 7> command{};
  std::array<hardware_interface::CommandInterface*, 6> effort{};
  auto& rm = manager->resources();
  for (size_t i = 0; i < 7; ++i)
  {
    const auto joint = "joint" + std::to_string(i + 1);
    StateAccess p(rm.claim_state_interface(joint + "/position"));
    state[i] = p.pointer();
    StateAccess v(rm.claim_state_interface(joint + "/velocity"));
    require(v.pointer()->set_value(0.), "set velocity");
    CommandAccess c(rm.claim_command_interface(joint + "/position"));
    command[i] = c.pointer();
    if (i < 6)
    {
      CommandAccess e(rm.claim_command_interface(joint + "/effort"));
      effort[i] = e.pointer();
    }
  }
  Values measured{ .02, .65, .85, .3, -.1, .1, .05 };
  auto inject = [&](bool initialize_command) {
    for (size_t i = 0; i < 7; ++i)
    {
      require(state[i]->set_value(measured[i]), "set measured");
      if (initialize_command)
        require(command[i]->set_value(measured[i]), "set initial command");
    }
  };
  inject(true);
  auto error = [&]() {
    double maximum = 0.;
    for (size_t i = 0; i < 7; ++i)
    {
      const double value = command[i]->get_optional().value();
      require(std::isfinite(value), "nonfinite command");
      maximum = std::max(maximum, std::abs(value - measured[i]));
    }
    return maximum;
  };
  const auto payload_com = settings["payload_com"].as<std::vector<double>>();
  kdl_gravity_compensation_controller::GravityModel expected_gravity(
      urdf, "base_link", "gripper_end", KDL::Vector(0, 0, -9.81), settings["payload_mass"].as<double>(),
      KDL::Vector(payload_com.at(0), payload_com.at(1), payload_com.at(2)));
  auto cycle = [&](bool process_callbacks = true) {
    const auto now = manager->get_trigger_clock()->now();
    manager->read(now, rclcpp::Duration::from_seconds(.01));
    for (size_t i = 0; i < 7; ++i)
      require(std::abs(state[i]->get_optional().value() - measured[i]) < 1e-12,
              "mock propagated command into measured state");
    require(manager->update(manager->now(), rclcpp::Duration::from_seconds(.01)) ==
                controller_interface::return_type::OK,
            "manager update failed");
    manager->write(now, rclcpp::Duration::from_seconds(.01));
    if (gravity->get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      KDL::JntArray q(6), tau(6);
      for (size_t i = 0; i < 6; ++i)
        q(i) = measured[i];
      require(expected_gravity.Compute(q, tau), "gravity model failed");
      for (size_t i = 0; i < 6; ++i)
        require(std::abs(effort[i]->get_optional().value() - tau(i)) < 1e-12,
                "active gravity command does not match nominal model plus fingers and unit gains");
    }
    if (process_callbacks)
      executor->spin_some();
    std::this_thread::sleep_for(10ms);
  };
  // Exercise the same controller-list lock as CM's diagnostics. Queue this
  // callback before each switch; it must run afterwards, never in its update pump.
  size_t callbacks = 0;
  rclcpp::TimerBase::SharedPtr list_callback;
  list_callback = manager->create_wall_timer(1ms, [&]() {
    require(manager->get_loaded_controllers().size() == 2, "queued controller-list callback failed");
    ++callbacks;
    list_callback->cancel();
  });
  auto await_callback = [&](size_t expected) {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (callbacks < expected && std::chrono::steady_clock::now() < deadline)
    {
      // spin_some collects work only once; a newly added timer can need a
      // further collection after the node's entity-notification waitable runs.
      executor->spin_some();
      std::this_thread::sleep_for(1ms);
    }
    require(callbacks == expected, "queued controller-list callback did not run");
  };
  await_callback(1);  // Prove the callback is registered before testing deferral.
  auto switch_controller = [&](bool activate) {
    list_callback->reset();
    const auto ready_deadline = std::chrono::steady_clock::now() + 1s;
    while (!list_callback->is_ready() && std::chrono::steady_clock::now() < ready_deadline)
      std::this_thread::sleep_for(1ms);
    require(list_callback->is_ready(), "controller-list callback was not queued");
    const auto callbacks_before = callbacks;
    auto future = std::async(std::launch::async, [&, activate]() {
      const auto start = fault == "--reject-switch" ? std::vector<std::string>{ "missing_test_controller" } :
                                                      std::vector<std::string>{ name, gravity_name };
      return manager->switch_controller(activate ? start : std::vector<std::string>{},
                                        activate ? std::vector<std::string>{} :
                                                   std::vector<std::string>{ name, gravity_name },
                                        2, false, rclcpp::Duration::from_seconds(3.));
    });
    std::exception_ptr cycle_failure;
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (future.wait_for(0s) != std::future_status::ready && std::chrono::steady_clock::now() < deadline)
    {
      try
      {
        if (cycle_failure)
        {
          // The test is already failed. Finish the in-flight CM operation before
          // rethrowing, rather than blocking in std::async's destructor. Do not
          // spin callbacks or discard the original assertion failure.
          const auto now = manager->get_trigger_clock()->now();
          manager->read(now, rclcpp::Duration::from_seconds(.01));
          manager->update(manager->now(), rclcpp::Duration::from_seconds(.01));
          manager->write(now, rclcpp::Duration::from_seconds(.01));
          std::this_thread::sleep_for(10ms);
        }
        else
        {
          require(fault != "--fail-during-switch", "injected failure during switch");
          // CM holds its controller-list lock while awaiting BOTH the update
          // handshake and the subsequent list-swap acknowledgement. spin_some()
          // here can block in CM diagnostics on that lock and starve this pump.
          // Match test_controller_chaining: only read/update/write until joined.
          cycle(false);
        }
      }
      catch (...)
      {
        if (!cycle_failure)
          cycle_failure = std::current_exception();
      }
    }
    if (future.wait_for(0s) != std::future_status::ready)
    {
      // No cancellation API exists for an in-flight CM switch. Never destroy
      // its borrowed fixture or block forever in the future destructor. This
      // memory-only test must fail, without claiming orderly cleanup or success.
      std::cerr << "switch timeout: worker did not quiesce; terminating test process\n";
      std::_Exit(2);
    }
    const auto result = future.get();
    std::cout << "Controller switch worker joined\n";
    if (cycle_failure)
      std::rethrow_exception(cycle_failure);
    require(result == controller_interface::return_type::OK, "manager switch failed");
    require(callbacks == callbacks_before, "executor callback ran inside switch update pump");
    await_callback(callbacks_before + 1);
  };
  switch_controller(true);
  require(fault != "--fail-after-activation", "injected failure after activation");
  auto node = std::make_shared<rclcpp::Node>("offline_gravity_mode_objectives");
  executor->add_node(node);
  auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false);
  context->robot_model =
      std::const_pointer_cast<moveit_pro::base::RobotModel>(controller->getRobotState().getRobotModel());
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
  BT::BehaviorTreeFactory factory;
  std::vector<std::shared_ptr<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>> plugins;
  for (auto plugin_name :
       { "moveit_pro::behaviors::CoreBehaviorsLoader", "moveit_pro::behaviors::MTCCoreBehaviorsLoader",
         "moveit_pro::behaviors::VisionBehaviorsLoader", "moveit_pro::behaviors::ConverterBehaviorsLoader",
         "rebot_b601_rs_hw::RebotHardwareBehaviorsLoader" })
  {
    auto p = loader.createSharedInstance(plugin_name);
    p->registerBehaviors(factory, context);
    plugins.push_back(p);
  }
  auto scene_service = node->create_service<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene", [&](const moveit_msgs::srv::GetPlanningScene::Request::SharedPtr,
                                 moveit_msgs::srv::GetPlanningScene::Response::SharedPtr response) {
        response->scene.robot_state.joint_state.name = joints;
        response->scene.robot_state.joint_state.position.assign(measured.begin(), measured.end());
        response->scene.robot_state.joint_state.velocity.assign(7, 0.);
        response->scene.robot_state.joint_state.header.stamp = node->now();
      });
  std::vector<GainService::Request> gains;
  std::string scenario;
  bool bad_order = false;
  auto gain_service = node->create_service<GainService>(
      "/robstride_hardware_interface/set_data_to_robstride",
      [&](const GainService::Request::SharedPtr request, GainService::Response::SharedPtr response) {
        const double command_error = error();
        const bool positive_kp = request->item_name == "kp" && request->item_data > 0;
        const bool ok = command_error < 1e-12 &&
                        controller->get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        bad_order |= !ok;
        std::cout << scenario << " BEFORE gain-service ACK id=" << static_cast<int>(request->id) << ' '
                  << request->item_name << '=' << request->item_data << " command_minus_measured_max=" << command_error
                  << (positive_kp ? " BEFORE_STIFFNESS" : "") << '\n';
        gains.push_back(*request);
        response->result = ok;
      });
  auto run = [&](const std::string& filename, const Values& pose) {
    scenario = filename;
    measured = pose;
    inject(false);
    gains.clear();
    bad_order = false;
    std::cout << scenario << " stale command error before explicit anchor=" << error() << '\n';
    require(error() > .2, "test does not begin with genuinely stale command");
    auto tree = factory.createTreeFromFile(hardware_share + "/objectives/" + filename);
    auto status = BT::NodeStatus::RUNNING;
    const auto deadline = std::chrono::steady_clock::now() + 12s;
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
    {
      status = tree.tickOnce();
      cycle();
    }
    if (status == BT::NodeStatus::RUNNING)
      tree.haltTree();
    require(status == BT::NodeStatus::SUCCESS,
            scenario + " did not succeed: " + context->logger->consumeErrorLogBuffer());
    require(gains.size() == 12 && !bad_order, scenario + " gain count/order failure");
    const std::array<double, 6> kd = filename == "rebot_set_float_gains.xml" ?
                                         std::array<double, 6>{ 1.5, 3.5, 5., 1.25, 1., 1. } :
                                         std::array<double, 6>{ 3., 10., 10., 5., 4., 4. };
    const std::array<double, 6> kp = filename == "rebot_set_float_gains.xml" ?
                                         std::array<double, 6>{} :
                                         std::array<double, 6>{ 50., 150., 150., 50., 50., 50. };
    for (size_t i = 0; i < 12; ++i)
    {
      require(gains[i].id == i % 6 + 1 && gains[i].item_name == (i < 6 ? "kd" : "kp"), "gain order mismatch");
      require(gains[i].item_data == (i < 6 ? kd[i] : kp[i - 6]), "wrong gain value");
    }
    require(controller->activations == 1 && controller->deactivations == 0, "Objective changed JTAC lifecycle");
    require(gravity->get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
            "gravity was deactivated");
    require(gravity->get_node()->get_parameter("gains").as_double_array() == std::vector<double>(6, 1.),
            "gravity gains changed");
    for (int i = 0; i < 3; ++i)
      cycle();
    require(error() < 1e-12, "anchor did not persist");
    std::cout << "PASS " << scenario
              << ": all 7 command positions equal measured BEFORE every gain write; JTAC stayed active.\n";
  };
  run("rebot_set_spring_gains.xml", { .15, 1.9, 2., -.7, .3, -.6, .2 });
  run("rebot_set_float_gains.xml", { -.15, 1.1, 1.3, .5, -.2, .6, .1 });
  run("rebot_set_spring_gains.xml", { .1, .8, 1.5, .2, .3, -.3, .15 });
  switch_controller(false);
  std::cout << "3/3 exact source gain-subtree runs PASS; 36 fake driver writes checked, 12 positive kp restorations "
               "preceded by exact measured commands.\n";
  executor->remove_node(node);
}

int main(int argc, char** argv)
{
  std::cout << std::unitbuf;
  rclcpp::init(argc, argv);
  int result = 0;
  try
  {
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    require(args.size() <= 2, "unexpected test arguments");
    const std::string fault = args.size() == 2 ? args[1] : "";
    require(fault.empty() || fault == "--fail-during-switch" || fault == "--fail-after-activation" ||
                fault == "--reject-switch",
            "unknown fault-injection argument");
    const auto urdf = readFile(TEST_MODE_URDF);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto manager = std::make_shared<Manager>(executor, urdf, true, "offline_cm");
    executor->add_node(manager);
    try
    {
      runModeTest(executor, manager, urdf, fault);
    }
    catch (const std::exception& error)
    {
      std::cerr << "Mode test failed: " << error.what() << '\n';
      result = 1;
    }
    // The switch worker has joined on both success and ordinary failure. Use
    // CM's public teardown before destroying the resource manager, while all
    // controller-owned interface loans still refer to live resources.
    require(manager->shutdown_controllers(), "controller shutdown failed");
    for (const auto& spec : manager->get_loaded_controllers())
      require(spec.c->get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
              "controller not finalized during cleanup");
    for (size_t i = 0; i < 7; ++i)
    {
      const auto joint = "joint" + std::to_string(i + 1);
      require(!manager->resources().command_interface_is_claimed(joint + "/position"), "position loan leaked");
      if (i < 6)
        require(!manager->resources().command_interface_is_claimed(joint + "/effort"), "effort loan leaked");
    }
    executor->remove_node(manager);
    manager.reset();
    executor.reset();
    std::cout << "Fixture cleanup complete: controllers finalized, 13 command loans released\n";
  }
  catch (const std::exception& error)
  {
    std::cerr << "Fixture failed: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  std::cout << "ROS context shutdown complete\n";
  return result;
}
