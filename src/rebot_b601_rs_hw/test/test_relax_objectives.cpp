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
#include <set>
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
  bool spring_fails = false;
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
    factory.registerSimpleAction("SpringGains", [this](BT::TreeNode&) {
      receipt.record("spring");
      return receipt.spring_fails ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    });
    factory.registerSimpleAction("TeleopEntry", [this](BT::TreeNode&) {
      receipt.record("teleop");
      return BT::NodeStatus::SUCCESS;
    });
    factory.registerSimpleAction("WaypointMotion", [this](BT::TreeNode&) {
      receipt.record("waypoint");
      return BT::NodeStatus::SUCCESS;
    });
    factory.registerSimpleAction("FloatGains", [this](BT::TreeNode&) {
      receipt.record("float");
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
      <BehaviorTree ID="reBot Set Spring Gains"><SpringGains/></BehaviorTree>
      <BehaviorTree ID="reBot Set Float Gains"><FloatGains/></BehaviorTree>
      <BehaviorTree ID="Move to Named Waypoint"><WaypointMotion/></BehaviorTree>
      <BehaviorTree ID="Move to Waypoint"><WaypointMotion/></BehaviorTree>
      </root>)");
    factory.registerBehaviorTreeFromFile(LEADER_OBJECTIVES "/rebot_check_leader_idle.xml");
    factory.registerBehaviorTreeFromFile(LEADER_OBJECTIVES "/rebot_notify_leader_home.xml");
    factory.registerBehaviorTreeFromFile(GENERATED_OBJECTIVES "/relax_at_home.xml");
    factory.registerBehaviorTreeFromFile(HW_SHARE "/objectives/rebot_engage_arm.xml");
    // Physical overrides of the stock Move to Raised/Rest, and the Boston waypoint entry.
    for (const auto* objective :
         { "teleoperate.xml", "move_to_raised.xml", "move_to_rest.xml", "start_gravity_comp.xml" })
      factory.registerBehaviorTreeFromFile(std::string(HW_SHARE "/objectives/") + objective);
    factory.registerBehaviorTreeFromFile(BOSTON_SHARE "/objectives/move_rebot_to_waypoint.xml");
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
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kIdle, kEngage, "switch", "spring", kEngage, "switch", "spring",
                                                       "HOME", kIdle, kRelax }));
}
TEST_F(RelaxObjectives, FailedSpringRestoreNeverMovesOrRelaxes)
{
  receipt.spring_fails = true;
  EXPECT_EQ(run("Relax at HOME"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kIdle, kEngage, "switch", "spring" }));
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
  EXPECT_EQ(receipt.events,
            (std::vector<std::string>{ kIdle, kEngage, "switch", "spring", kEngage, "switch", "spring", "HOME" }));
}
TEST_F(RelaxObjectives, RefusedRelaxIsFailureNotBestEffort)
{
  receipt.refused = kRelax;
  EXPECT_EQ(run("Relax at HOME"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kIdle, kEngage, "switch", "spring", kEngage, "switch", "spring",
                                                       "HOME", kIdle, kRelax }));
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
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kEngage, "switch", "spring", "teleop" }));
}
TEST_F(RelaxObjectives, TeleopFailedSpringRestoreNeverEnters)
{
  receipt.spring_fails = true;
  EXPECT_EQ(run("Teleoperate"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kEngage, "switch", "spring" }));
}
TEST_F(RelaxObjectives, HomeFailedSpringRestoreNeverMovesOrRelaxes)
{
  receipt.spring_fails = true;
  EXPECT_EQ(run("Relax at HOME"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kIdle, kEngage, "switch", "spring" }));
}
TEST_F(RelaxObjectives, MirrorEngagesAndRestoresSpringBeforePrepareAndControllerSwitch)
{
  auto tree = factory.createTree("Mirror reBot Leader");
  for (int i = 0; i < 100 && receipt.snapshot().size() < 6; ++i)
  {
    tree.tickExactlyOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  tree.haltTree();
  const auto events = receipt.snapshot();
  ASSERT_GE(events.size(), 6U);
  EXPECT_EQ(std::vector<std::string>(events.begin(), events.begin() + 6),
            (std::vector<std::string>{ kEngage, "switch", "spring", "/rebot_leader_bridge/prepare", "switch",
                                       "/rebot_leader_bridge/mirror" }));
}
TEST_F(RelaxObjectives, MirrorFailedSpringRestoreNeverPrepares)
{
  receipt.spring_fails = true;
  EXPECT_EQ(run("Mirror reBot Leader"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kEngage, "switch", "spring" }));
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
// After Relax at HOME torque is off: every arm motion entry engages, owns JTAC
// and restores spring gains before it moves, and never moves on a refused engage.
class EngageFirstMotion : public RelaxObjectives, public testing::WithParamInterface<const char*>
{
};
TEST_P(EngageFirstMotion, EngagesSwitchesAndRestoresSpringBeforeMotion)
{
  EXPECT_EQ(run(GetParam()), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kEngage, "switch", "spring", "waypoint" }));
}
TEST_P(EngageFirstMotion, RefusedEngageNeverMoves)
{
  receipt.refused = kEngage;
  EXPECT_EQ(run(GetParam()), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kEngage }));
}
TEST_P(EngageFirstMotion, MissingEngageNeverMoves)
{
  receipt.unavailable = kEngage;
  EXPECT_EQ(run(GetParam()), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(receipt.events.empty());
}
TEST_P(EngageFirstMotion, FailedSpringRestoreNeverMoves)
{
  receipt.spring_fails = true;
  EXPECT_EQ(run(GetParam()), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kEngage, "switch", "spring" }));
}
INSTANTIATE_TEST_SUITE_P(Waypoints, EngageFirstMotion,
                         testing::Values("Move to Raised", "Move to Rest", "Move reBot to Waypoint"));

TEST_F(RelaxObjectives, StartGravityCompEngagesBeforeFloat)
{
  EXPECT_EQ(run("Start Gravity Comp"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kEngage, "switch", "float" }));
}
TEST_F(RelaxObjectives, StartGravityCompRefusedEngageNeverFloats)
{
  receipt.refused = kEngage;
  EXPECT_EQ(run("Start Gravity Comp"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kEngage }));
}

// Record and Replay reBot Motion with the real engage, gravity-comp and trigger
// subtrees; recorder, files, UI prompts and motion are test seams.
class RecordReplay : public RelaxObjectives
{
protected:
  void SetUp() override
  {
    RelaxObjectives::SetUp();
    const auto port = [](const char* name) { return BT::InputPort<std::string>(name); };
    const auto out = [](const char* name) { return BT::OutputPort<std::string>(name); };
    factory.registerSimpleAction("GetFilenamesFromDirectory",
                                 [this](BT::TreeNode& node) {
                                   const auto folder = node.getInput<std::string>("directory_path").value();
                                   receipt.record("list:" + folder);
                                   return folder == missing_folder ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
                                 },
                                 { port("directory_path"), port("file_type"), out("result") });
    factory.registerSimpleAction("LoadJointTrajectoryFromYaml",
                                 [this](BT::TreeNode& node) {
                                   const auto path = node.getInput<std::string>("file_path").value();
                                   receipt.record("load:" + path);
                                   return files.contains(path) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
                                 },
                                 { port("file_path"), out("output") });
    factory.registerSimpleAction("LogMessage",
                                 [this](BT::TreeNode& node) {
                                   receipt.record("log:" + node.getInput<std::string>("log_level").value());
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { port("message"), port("log_level") });
    factory.registerSimpleDecorator("SuppressChildErrors", [](BT::NodeStatus status, BT::TreeNode&) { return status; });
    factory.registerSimpleAction("WaitForUserDecision",
                                 [this](BT::TreeNode& node) {
                                   receipt.record("ask:" + node.getInput<std::string>("approve_label").value());
                                   if (answers.empty())
                                     return BT::NodeStatus::FAILURE;
                                   const bool approve = answers.front();
                                   answers.erase(answers.begin());
                                   return approve ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
                                 },
                                 { port("prompt"), port("approve_label"), port("reject_label"), port("timeout_sec") });
    factory.registerSimpleAction("RecordJointTrajectory",
                                 [this](BT::TreeNode& node) {
                                   receipt.record("record:" + node.getInput<std::string>("planning_group_name").value());
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { port("joint_state_topic"), out("joint_trajectory_message"),
                                   port("planning_group_name"), port("start_recording_service"),
                                   port("stop_recording_service"), port("timeout_sec"), port("joint_space_step") });
    factory.registerSimpleAction("SaveJointTrajectoryToYaml",
                                 [this](BT::TreeNode& node) {
                                   receipt.record("save:" + node.getInput<std::string>("yaml_filename").value());
                                   return save_fails ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
                                 },
                                 { port("message"), port("yaml_filename"), port("overwrite") });
    factory.registerSimpleAction("GetCurrentPlanningScene", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                                 { out("planning_scene_msg") });
    factory.registerSimpleAction("ValidateTrajectory",
                                 [this](BT::TreeNode&) {
                                   receipt.record("validate");
                                   return invalid ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
                                 },
                                 { port("planning_scene_msg"), port("planning_group_name"), port("joint_trajectory_msg"),
                                   port("joint_space_step"), port("cartesian_space_step") });
    factory.registerSimpleAction("GetRobotStateFromTrajectory", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                                 { port("joint_trajectory_msg"), port("from_start"), port("time_from_reference"),
                                   out("joint_state") });
    factory.registerSimpleAction("ApproachMotion", [this](BT::TreeNode&) {
      receipt.record("approach");
      return BT::NodeStatus::SUCCESS;
    });
    factory.registerSimpleAction("ExecuteTrajectory",
                                 [this](BT::TreeNode&) {
                                   receipt.record("execute");
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { port("joint_trajectory_msg"), port("execution_pipeline"),
                                   port("controller_action_name") });
    factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4">
      <BehaviorTree ID="Move to Joint State"><ApproachMotion/></BehaviorTree></root>)");
    for (const auto* objective : { "stop_gravity_comp.xml", "record_rebot_motion.xml", "replay_rebot_motion.xml" })
      factory.registerBehaviorTreeFromFile(std::string(HW_SHARE "/objectives/") + objective);
  }
  BT::NodeStatus runWith(const std::string& name, const std::string& motion = "wave",
                         const std::string& overwrite = "false")
  {
    auto blackboard = BT::Blackboard::create();
    blackboard->set<std::string>("recordings_folder", kFolder);
    blackboard->set<std::string>("motion_name", motion);
    blackboard->set<std::string>("overwrite", overwrite);
    blackboard->set<std::string>("joint_space_step", "0.01");
    blackboard->set<std::string>("max_recording_seconds", "600.0");
    auto tree = factory.createTree(name, blackboard);
    return tree.tickWhileRunning();
  }
  static constexpr auto kFolder = "/data/rebot_recordings";
  const std::string kPath = std::string(kFolder) + "/wave.yaml";
  std::set<std::string> files;
  std::vector<bool> answers;
  std::string missing_folder;
  bool save_fails = false;
  bool invalid = false;
};

const std::string kStart = "/record_rebot_motion/start";
const std::string kStop = "/record_rebot_motion/stop";
const std::string kList = "list:/data/rebot_recordings";

TEST_F(RecordReplay, RecordFloatsOnlyAfterStartAndSavesAfterSpringRestore)
{
  answers = { true, true };
  EXPECT_EQ(runWith("Record reBot Motion"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(receipt.events,
            (std::vector<std::string>{ kList, "load:" + kPath, kEngage, "switch", "spring", "ask:Start recording",
                                       kEngage, "switch", "float", "record:manipulator_and_gripper", kStart,
                                       "ask:Stop recording", kStop, "switch", "spring", "save:" + kPath }));
}
TEST_F(RecordReplay, DiscardRestoresSpringAndSavesNothing)
{
  answers = { true, false };
  EXPECT_EQ(runWith("Record reBot Motion"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events,
            (std::vector<std::string>{ kList, "load:" + kPath, kEngage, "switch", "spring", "ask:Start recording",
                                       kEngage, "switch", "float", "record:manipulator_and_gripper", kStart,
                                       "ask:Stop recording", "switch", "spring", "log:error" }));
}
TEST_F(RecordReplay, CancelAtStartNeverFloats)
{
  answers = { false };
  EXPECT_EQ(runWith("Record reBot Motion"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kList, "load:" + kPath, kEngage, "switch", "spring",
                                                       "ask:Start recording", "log:error" }));
}
TEST_F(RecordReplay, FailedSaveRestoresSpring)
{
  answers = { true, true };
  save_fails = true;
  EXPECT_EQ(runWith("Record reBot Motion"), BT::NodeStatus::FAILURE);
  const auto events = receipt.snapshot();
  ASSERT_GE(events.size(), 4U);
  EXPECT_EQ(std::vector<std::string>(events.end() - 4, events.end()),
            (std::vector<std::string>{ "save:" + kPath, "switch", "spring", "log:error" }));
}
TEST_F(RecordReplay, RefusesBeforeEngageOnEmptyNameMissingFolderOrTakenName)
{
  // An empty run-time string input arrives as 'null'.
  for (const auto* empty : { "", "null" })
  {
    EXPECT_EQ(runWith("Record reBot Motion", empty), BT::NodeStatus::FAILURE);
    EXPECT_EQ(runWith("Replay reBot Motion", empty), BT::NodeStatus::FAILURE);
  }
  EXPECT_EQ(receipt.snapshot(), std::vector<std::string>(4, "log:error"));
  receipt.events.clear();
  missing_folder = kFolder;
  EXPECT_EQ(runWith("Record reBot Motion"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.snapshot(), (std::vector<std::string>{ kList }));
  missing_folder.clear();
  receipt.events.clear();
  files = { kPath };
  EXPECT_EQ(runWith("Record reBot Motion"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.snapshot(), (std::vector<std::string>{ kList, "load:" + kPath, "log:error" }));
}
TEST_F(RecordReplay, OverwriteSkipsTheTakenNameCheck)
{
  files = { kPath };
  answers = { true, true };
  EXPECT_EQ(runWith("Record reBot Motion", "wave", "true"), BT::NodeStatus::SUCCESS);
  const auto events = receipt.snapshot();
  ASSERT_GE(events.size(), 2U);
  EXPECT_EQ(events[1], kEngage);
  EXPECT_EQ(events.back(), "save:" + kPath);
}
TEST_F(RecordReplay, RecordRefusedEngageNeverAsksOrFloats)
{
  receipt.refused = kEngage;
  answers = { true, true };
  EXPECT_EQ(runWith("Record reBot Motion"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ kList, "load:" + kPath, kEngage }));
}
TEST_F(RecordReplay, ReplayEngagesChecksApproachesAndExecutesWithoutHomeOrRelax)
{
  files = { kPath };
  EXPECT_EQ(runWith("Replay reBot Motion"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ "load:" + kPath, kEngage, "switch", "spring", "validate",
                                                       "approach", "execute" }));
}
TEST_F(RecordReplay, ReplayMissingRecordingNeverEngages)
{
  EXPECT_EQ(runWith("Replay reBot Motion"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ "load:" + kPath }));
}
TEST_F(RecordReplay, ReplayInvalidRecordingNeverMoves)
{
  files = { kPath };
  invalid = true;
  EXPECT_EQ(runWith("Replay reBot Motion"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ "load:" + kPath, kEngage, "switch", "spring", "validate" }));
}
TEST_F(RecordReplay, ReplayRefusedEngageNeverMoves)
{
  files = { kPath };
  receipt.refused = kEngage;
  EXPECT_EQ(runWith("Replay reBot Motion"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(receipt.events, (std::vector<std::string>{ "load:" + kPath, kEngage }));
}
}  // namespace
