// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
// Real rc10 CallTriggerService; only external motion/leader/UI are test seams.
#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <tinyxml2.h>
#include <moveit_pro_behavior/behaviors/core/call_trigger_service.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_pro_test_utils/ros_test.hpp>

#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
using Trigger = std_srvs::srv::Trigger;
using namespace moveit_pro::behaviors;

struct Receipt
{
  std::vector<std::string> events;
  std::string refused;
  std::string unavailable;
  bool home_fails = false;
  std::mutex mutex;

  void record(const std::string& event)
  {
    std::lock_guard lock(mutex);
    events.push_back(event);
  }
  std::vector<std::string> snapshot()
  {
    std::lock_guard lock(mutex);
    return events;
  }
};

class FakeTrigger final : public ClientInterfaceBase<Trigger>
{
public:
  explicit FakeTrigger(Receipt& receipt) : receipt_(receipt)
  {
  }
  void initialize(const std::string& service, std::chrono::duration<double>, std::chrono::duration<double>) override
  {
    service_ = service;
  }
  bool waitForServiceServer() const override
  {
    return service_ != receipt_.unavailable;
  }
  tl::expected<Trigger::Response, std::string> syncSendRequest(const Trigger::Request&) override
  {
    receipt_.record(service_);
    Trigger::Response response;
    response.success = service_ != receipt_.refused;
    response.message = response.success ? "fake measured HOME / ms=20 max_step_rad=0" : "guard refused";
    return response;
  }
  void cancelRequest() override
  {
  }

private:
  Receipt& receipt_;
  std::string service_;
};

std::string file(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("Cannot read " + path);
  std::ostringstream text;
  text << stream.rdbuf();
  return text.str();
}

class RelaxObjectives : public moveit_pro::test_utils::RosTest
{
protected:
  void SetUp() override
  {
    context = std::make_shared<BehaviorContext>(std::make_shared<rclcpp::Node>("test_relax_objectives"), false);
    factory.registerBuilder(BT::CreateManifest<CallTriggerService>("CallTriggerService"),
                            [this](const std::string& name, const BT::NodeConfig& config) {
                              return std::make_unique<CallTriggerService>(name, config, context,
                                                                          std::make_unique<FakeTrigger>(receipt));
                            });
    factory.registerSimpleAction("HomeMotion", [this](BT::TreeNode&) {
      receipt.record("HOME");
      return receipt.home_fails ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    });
    factory.registerSimpleAction("TeleopEntry", [this](BT::TreeNode&) {
      receipt.record("teleop");
      return BT::NodeStatus::SUCCESS;
    });
    factory.registerSimpleAction("CallRebotLeaderService",
                                 [this](BT::TreeNode& node) {
                                   receipt.record(node.getInput<std::string>("service_name").value());
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::InputPort<std::string>("service_name") });
    factory.registerSimpleAction("SwitchController",
                                 [this](BT::TreeNode&) {
                                   receipt.record("switch");
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::InputPort<std::string>("activate_controllers"),
                                   BT::InputPort<bool>("automatic_deactivation"), BT::InputPort<int>("strictness") });
    factory.registerSimpleAction("WaitForDuration", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                                 { BT::InputPort<double>("delay_duration") });
    // Keep the shipped public HOME wrapper and its exact engage gate. Only the
    // internal powered motion is replaced; test_home_recovery exercises it fully.
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.Parse(file(GENERATED_OBJECTIVES "/return_rebot_leader_home.xml").c_str()), tinyxml2::XML_SUCCESS);
    auto* root = doc.RootElement();
    auto* wrapper = root->FirstChildElement("BehaviorTree");
    root->DeleteChild(wrapper->NextSiblingElement("BehaviorTree"));
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    factory.registerBehaviorTreeFromText(printer.CStr());
    factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4">
      <BehaviorTree ID="Return reBot Leader Home Under Power"><HomeMotion/></BehaviorTree>
      <BehaviorTree ID="Request Teleoperation"><TeleopEntry/></BehaviorTree>
      </root>)");
    factory.registerBehaviorTreeFromFile(LEADER_OBJECTIVES "/rebot_check_leader_idle.xml");
    factory.registerBehaviorTreeFromFile(LEADER_OBJECTIVES "/rebot_notify_leader_home.xml");
    factory.registerBehaviorTreeFromFile(GENERATED_OBJECTIVES "/relax_at_home.xml");
    factory.registerBehaviorTreeFromFile(HW_SHARE "/objectives/teleoperate.xml");
    factory.registerBehaviorTreeFromFile(LEADER_OBJECTIVES "/mirror_rebot_leader.xml");
  }
  BT::NodeStatus run(const std::string& name)
  {
    auto tree = factory.createTree(name);
    return tree.tickWhileRunning();
  }
  Receipt receipt;
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<BehaviorContext> context;
};

const std::string kEngage = "/robstride_hardware_interface/engage";
const std::string kRelax = "/robstride_hardware_interface/relax";
const std::string kIdle = "/rebot_leader_bridge/idle";
const std::string kNotify = "/rebot_leader_bridge/home_verified";

TEST_F(RelaxObjectives, LeaderHomeNotificationUsesRealTrigger)
{
  EXPECT_EQ(run("reBot Notify Leader Home"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kNotify }));
}
TEST_F(RelaxObjectives, RefusedLeaderHomeNotificationRemainsBestEffort)
{
  receipt.refused = kNotify;
  EXPECT_EQ(run("reBot Notify Leader Home"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kNotify }));
}
TEST_F(RelaxObjectives, MissingLeaderHomeNotificationRemainsBestEffort)
{
  receipt.unavailable = kNotify;
  EXPECT_EQ(run("reBot Notify Leader Home"), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(receipt.events.empty());
}
TEST_F(RelaxObjectives, StockHomeNotificationMakesNoServiceCall)
{
  factory.registerBehaviorTreeFromFile(HW_SHARE "/objectives/rebot_notify_leader_home.xml");
  EXPECT_EQ(run("reBot Notify Leader Home"), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(receipt.events.empty());
}

TEST_F(RelaxObjectives, HomeBeforeRelaxUsesRealTriggerSuccess)
{
  EXPECT_EQ(run("Relax at HOME"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kIdle, kEngage, "HOME", kIdle, kRelax }));
}
TEST_F(RelaxObjectives, RefusedEngageNeverMovesOrRelaxes)
{
  receipt.refused = kEngage;
  EXPECT_EQ(run("Relax at HOME"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kIdle, kEngage }));
}
TEST_F(RelaxObjectives, FailedHomeNeverCallsRelax)
{
  receipt.home_fails = true;
  EXPECT_EQ(run("Relax at HOME"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kIdle, kEngage, "HOME" }));
}
TEST_F(RelaxObjectives, RefusedRelaxIsFailureNotBestEffort)
{
  receipt.refused = kRelax;
  EXPECT_EQ(run("Relax at HOME"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kIdle, kEngage, "HOME", kIdle, kRelax }));
}
TEST_F(RelaxObjectives, MissingEngageNeverMoves)
{
  receipt.unavailable = kEngage;
  EXPECT_EQ(run("Relax at HOME"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kIdle }));
}
TEST_F(RelaxObjectives, FollowingLeaderRefusesBeforeHomeOrRelax)
{
  receipt.refused = kIdle;
  EXPECT_EQ(run("Relax at HOME"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kIdle }));
}
TEST_F(RelaxObjectives, MissingIdleServiceFailsClosed)
{
  receipt.unavailable = kIdle;
  EXPECT_EQ(run("Relax at HOME"), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(receipt.events.empty());
}
TEST_F(RelaxObjectives, TeleopWaitsForSuccessfulEngage)
{
  EXPECT_EQ(run("Teleoperate"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kEngage, "teleop" }));
}
TEST_F(RelaxObjectives, MirrorEngagesBeforePrepareAndControllerSwitch)
{
  auto tree = factory.createTree("Mirror reBot Leader");
  for (int i = 0; i < 100 && receipt.snapshot().size() < 4; ++i)
  {
    tree.tickExactlyOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  tree.haltTree();
  const auto events = receipt.snapshot();
  ASSERT_GE(events.size(), 4U);
  EXPECT_EQ(
      std::vector<std::string>(events.begin(), events.begin() + 4),
      (std::vector<std::string>{ kEngage, "/rebot_leader_bridge/prepare", "switch", "/rebot_leader_bridge/mirror" }));
}
TEST_F(RelaxObjectives, MirrorRefusedEngageNeverPreparesOrSwitches)
{
  receipt.refused = kEngage;
  EXPECT_EQ(run("Mirror reBot Leader"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kEngage }));
}
TEST_F(RelaxObjectives, TeleopRefusedEngageNeverEnters)
{
  receipt.refused = kEngage;
  EXPECT_EQ(run("Teleoperate"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kEngage }));
}
}  // namespace
