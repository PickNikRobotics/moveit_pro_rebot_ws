// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_pro_test_utils/ros_test.hpp>
#include <pluginlib/class_loader.hpp>
#include <rebot_b601_rs_hw/set_rebot_motor_gain.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <utility>

using namespace std::chrono_literals;
using GainService = robstride_interfaces::srv::SetDataToRobstride;

namespace
{
class GainClient : public moveit_pro::behaviors::ClientInterfaceBase<GainService>
{
public:
  void initialize(const std::string& name, std::chrono::duration<double> server_timeout,
                  std::chrono::duration<double> response_timeout) override
  {
    service = name;
    wait = server_timeout;
    timeout = response_timeout;
  }
  bool waitForServiceServer() const override
  {
    return available;
  }
  tl::expected<GainService::Response, std::string> syncSendRequest(const GainService::Request& value) override
  {
    request = value;
    started = true;
    while (block && !canceled)
      std::this_thread::sleep_for(1ms);
    if (!error.empty())
      return tl::make_unexpected(error);
    return response;
  }
  void cancelRequest() override
  {
    canceled = true;
  }
  GainService::Request request;
  GainService::Response response;
  std::string service, error;
  std::chrono::duration<double> wait{}, timeout{};
  bool available = true;
  std::atomic<bool> started{ false }, block{ false }, canceled{ false };
};

class MotorGainTest : public moveit_pro::test_utils::RosTest
{
protected:
  void SetUp() override
  {
    context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(
        std::make_shared<rclcpp::Node>("motor_gain_test"), false);
    config.blackboard = BT::Blackboard::create();
    config.input_ports = { { "motor_id", "2" }, { "gain", "kd" }, { "value", "3.5" } };
  }
  void create()
  {
    auto client = std::make_unique<GainClient>();
    fake = client.get();
    fake->response.result = true;
    behavior = std::make_unique<rebot_b601_rs_hw::SetRebotMotorGain>("gain", config, context, std::move(client));
  }
  BT::NodeStatus run(rclcpp::Executor* executor = nullptr)
  {
    const auto end = std::chrono::steady_clock::now() + 4s;
    BT::NodeStatus status;
    do
    {
      status = behavior->executeTick();
      if (executor)
        executor->spin_some();
      std::this_thread::sleep_for(1ms);
    } while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < end);
    if (status == BT::NodeStatus::RUNNING)
      behavior->haltNode();
    return status;
  }
  BT::NodeConfiguration config;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context;
  GainClient* fake = nullptr;
  std::unique_ptr<rebot_b601_rs_hw::SetRebotMotorGain> behavior;
};

TEST_F(MotorGainTest, QueuesExactArmGainWithBoundedTimeout)
{
  create();
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fake->service, "/robstride_hardware_interface/set_data_to_robstride");
  EXPECT_EQ(fake->wait, 3s);
  EXPECT_EQ(fake->timeout, 500ms);
  EXPECT_EQ(fake->request.id, 2);
  EXPECT_EQ(fake->request.item_name, "kd");
  EXPECT_FLOAT_EQ(fake->request.item_data, 3.5f);
  EXPECT_GT(rclcpp::Time(fake->request.header.stamp).nanoseconds(), 0);
}

TEST_F(MotorGainTest, EveryRequiredPortMustBePresent)
{
  for (const auto* port : { "motor_id", "gain" })
  {
    SCOPED_TRACE(port);
    const auto saved = config.input_ports.at(port);
    config.input_ports.erase(port);
    create();
    EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(fake->started);
    config.input_ports[port] = saved;
  }
}

// A RobStride description as the driver config publishes it: the jaw GPIO comes
// first and carries a non-stock jaw_kp, so neither order nor a stock constant passes.
std::string description(const std::string& extra_control = "")
{
  return R"(<robot name="rebot"><ros2_control name="RebotSystem" type="system">
    <hardware><plugin>robstride_hardware_interface/RobstrideHardware</plugin></hardware>
    <gpio name="motor7"><param name="ID">7</param><param name="kp">10.0</param><param name="kd">0.7</param></gpio>
    <gpio name="motor2"><param name="ID">2</param><param name="kp">150.0</param><param name="kd">10.0</param></gpio>
    </ros2_control>)" +
         extra_control + "</robot>";
}

class ConfiguredGainTest : public MotorGainTest
{
protected:
  BT::NodeStatus runConfigured(const std::string& published)
  {
    auto publisher_node = std::make_shared<rclcpp::Node>("fake_robot_state_publisher");
    auto publisher = publisher_node->create_publisher<std_msgs::msg::String>(
        "/robot_description", rclcpp::QoS(1).reliable().transient_local());
    std_msgs::msg::String message;
    message.data = published;
    publisher->publish(message);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(publisher_node);
    executor.add_node(context->node);
    config.input_ports.erase("value");
    create();
    return run(&executor);
  }
};

TEST_F(ConfiguredGainTest, OmittedValueWritesTheConfiguredJawKp)
{
  config.input_ports = { { "motor_id", "7" }, { "gain", "kp" } };
  EXPECT_EQ(runConfigured(description()), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fake->request.id, 7);
  EXPECT_EQ(fake->request.item_name, "kp");
  EXPECT_FLOAT_EQ(fake->request.item_data, 10.0f);
}

TEST_F(ConfiguredGainTest, OmittedValueUsesTheGpioIdAndGain)
{
  EXPECT_EQ(runConfigured(description()), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fake->request.id, 2);
  EXPECT_EQ(fake->request.item_name, "kd");
  EXPECT_FLOAT_EQ(fake->request.item_data, 10.0f);
}

TEST_F(ConfiguredGainTest, AmbiguousOrMissingConfigurationNeverReachesDriver)
{
  const std::string second_driver = R"(<ros2_control name="Other" type="system">
    <hardware><plugin>robstride_hardware_interface/RobstrideHardware</plugin></hardware></ros2_control>)";
  const std::string jaw_without_kp = R"(<robot name="rebot"><ros2_control name="RebotSystem" type="system">
    <hardware><plugin>robstride_hardware_interface/RobstrideHardware</plugin></hardware>
    <gpio name="motor7"><param name="ID">7</param><param name="kd">0.7</param></gpio></ros2_control></robot>)";
  for (const auto& [motor, published] :
       { std::pair{ "7", std::string("not xml") }, std::pair{ "7", description(second_driver) },
         std::pair{ "4", description() }, std::pair{ "7", jaw_without_kp } })
  {
    SCOPED_TRACE(published);
    config.input_ports = { { "motor_id", motor }, { "gain", "kp" } };
    EXPECT_EQ(runConfigured(published), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(fake->started);
  }
}

// The MuJoCo twin's ros2_control block (rebot.urdf.xacro, hardware_interface:=mujoco)
// configures no RobStride motor, so there is no jaw float to end and nothing to write.
TEST_F(ConfiguredGainTest, DescriptionWithoutRobstrideHardwareWritesNothingAndSucceeds)
{
  config.input_ports = { { "motor_id", "7" }, { "gain", "kp" } };
  EXPECT_EQ(runConfigured(R"(<robot name="rebot"><ros2_control name="RebotSystem" type="system">
    <hardware><plugin>picknik_mujoco_ros/MujocoSystem</plugin></hardware></ros2_control></robot>)"),
            BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(fake->started);
}

TEST_F(MotorGainTest, OmittedValueWithoutRobotDescriptionNeverReachesDriver)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(context->node);
  config.input_ports = { { "motor_id", "7" }, { "gain", "kp" } };
  create();
  EXPECT_EQ(run(&executor), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(fake->started);
  EXPECT_NE(context->logger->consumeErrorLogBuffer().find("/robot_description"), std::string::npos);
}

TEST_F(MotorGainTest, QueuesJawGainWithRs00Bounds)
{
  config.input_ports = { { "motor_id", "7" }, { "gain", "kp" }, { "value", "0.0" } };
  create();
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fake->request.id, 7);
  EXPECT_EQ(fake->request.item_name, "kp");
  EXPECT_FLOAT_EQ(fake->request.item_data, 0.0f);
  for (const auto& [gain, value] : { std::pair{ "kp", "500.01" }, std::pair{ "kd", "5.01" } })
  {
    SCOPED_TRACE(gain);
    config.input_ports = { { "motor_id", "7" }, { "gain", gain }, { "value", value } };
    create();
    EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(fake->started);
  }
}

TEST_F(MotorGainTest, InvalidMotorIdCannotReachDriver)
{
  for (const auto* value : { "-1", "0", "8", "256", "motor2" })
  {
    SCOPED_TRACE(value);
    config.input_ports["motor_id"] = value;
    create();
    EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(fake->started);
  }
}

TEST_F(MotorGainTest, RejectsArbitraryRegisters)
{
  for (const auto* value : { "", "0x700B", "position", "KP", "gains" })
  {
    SCOPED_TRACE(value);
    config.input_ports["gain"] = value;
    create();
    EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(fake->started);
  }
}

TEST_F(MotorGainTest, RejectsNonfiniteNegativeAndOversizedGains)
{
  config.input_ports["value"] = "{gain_value}";
  for (double value :
       { -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), 100.01 })
  {
    SCOPED_TRACE(value);
    config.blackboard->set("gain_value", value);
    create();
    EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(fake->started);
  }
  config.input_ports["motor_id"] = "4";
  config.blackboard->set("gain_value", 5.01);
  create();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(fake->started);
  config.input_ports["gain"] = "kp";
  config.blackboard->set("gain_value", 500.01);
  create();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(fake->started);
  config.input_ports["motor_id"] = "1";
  config.blackboard->set("gain_value", 5000.01);
  create();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(fake->started);
}

TEST_F(MotorGainTest, AcceptsZeroAndProtocolBoundariesForEveryArmMotor)
{
  for (int id = 1; id <= 6; ++id)
    for (const auto* gain : { "kp", "kd" })
      for (double value : { 0.0, std::string(gain) == "kp" ? (id <= 3 ? 5000.0 : 500.0) : (id <= 3 ? 100.0 : 5.0) })
      {
        config.input_ports["motor_id"] = std::to_string(id);
        config.input_ports["gain"] = gain;
        config.input_ports["value"] = std::to_string(value);
        create();
        EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
        EXPECT_EQ(fake->request.id, id);
        EXPECT_EQ(fake->request.item_name, gain);
        EXPECT_FLOAT_EQ(fake->request.item_data, static_cast<float>(value));
      }
}

TEST_F(MotorGainTest, RejectionIsVisibleInObjectiveFailureLog)
{
  create();
  fake->response.result = false;
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_NE(context->logger->consumeErrorLogBuffer().find("driver rejected"), std::string::npos);
}

TEST_F(MotorGainTest, UnavailableServiceSendsNoRequest)
{
  create();
  fake->available = false;
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(fake->started);
}

TEST_F(MotorGainTest, TransportTimeoutIsFailureNotAnAcknowledgement)
{
  create();
  fake->error = "response timed out";
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_NE(context->logger->consumeErrorLogBuffer().find("response timed out"), std::string::npos);
}

TEST_F(MotorGainTest, HaltCancelsWaitWithoutSendingCompensatingWrites)
{
  create();
  fake->block = true;
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::RUNNING);
  const auto end = std::chrono::steady_clock::now() + 1s;
  while (!fake->started && std::chrono::steady_clock::now() < end)
    std::this_thread::sleep_for(1ms);
  if (!fake->started)
  {
    fake->block = false;
    FAIL() << "Request did not start";
  }
  behavior->haltNode();
  EXPECT_TRUE(fake->canceled);
  EXPECT_EQ(fake->request.item_name, "kd");
}

TEST_F(MotorGainTest, RealRosClientTalksOnlyToFakeService)
{
  auto server = std::make_shared<rclcpp::Node>("fake_rebot_gain_driver");
  GainService::Request received;
  bool called = false;
  auto service = server->create_service<GainService>("/robstride_hardware_interface/set_data_to_robstride",
                                                     [&received, &called](const GainService::Request::SharedPtr request,
                                                                          GainService::Response::SharedPtr response) {
                                                       received = *request;
                                                       called = true;
                                                       response->result = true;
                                                     });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(context->node);
  behavior = std::make_unique<rebot_b601_rs_hw::SetRebotMotorGain>("gain", config, context);
  EXPECT_EQ(run(&executor), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(called);
  EXPECT_EQ(received.id, 2);
  EXPECT_EQ(received.item_name, "kd");
  EXPECT_FLOAT_EQ(received.item_data, 3.5f);
}

TEST_F(MotorGainTest, ConfiguredPluginLoaderInstantiatesExactId)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
  auto plugin = loader.createSharedInstance("rebot_b601_rs_hw::RebotHardwareBehaviorsLoader");
  BT::BehaviorTreeFactory factory;
  plugin->registerBehaviors(factory, context);
  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4"><BehaviorTree ID="test">
      <Action ID="SetRebotMotorGain" motor_id="2" gain="kd" value="3.5"/>
    </BehaviorTree></root>)");
  ASSERT_NE(tree.rootNode(), nullptr);
  EXPECT_EQ(tree.rootNode()->registrationName(), "SetRebotMotorGain");
}
}  // namespace
