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
// Original recorder cases from test_anchor_trajectory_to_command.cpp.
// The seven generic anchoring cases moved to motion_recovery_behaviors.
#include <behaviortree_cpp/bt_factory.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rebot_bench_behaviors/record_rebot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace rebot_bench_behaviors
{
namespace
{
using ::testing::HasSubstr;

trajectory_msgs::msg::JointTrajectory plannedTrajectory()
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = { "joint1", "joint2" };
  const auto add = [&](const int seconds, const std::vector<double>& positions, const std::vector<double>& velocities) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = seconds;
    point.positions = positions;
    point.velocities = velocities;
    point.accelerations = { 0.0, 0.0 };
    trajectory.points.push_back(point);
  };
  add(0, { 1.0, 2.0 }, { 0.0, 0.0 });
  add(1, { 1.1, 1.95 }, { 0.15, -0.075 });
  add(2, { 1.2, 1.9 }, { 0.0, 0.0 });
  return trajectory;
}

class RecordRebotTrajectoryTest : public ::testing::Test
{
protected:
  RecordRebotTrajectoryTest()
    : node_(std::make_shared<rclcpp::Node>("test_record_rebot_trajectory"))
    , context_(std::make_shared<moveit_pro::behaviors::BehaviorContext>(node_, false))
  {
    moveit_pro::behaviors::registerBehavior<RecordRebotTrajectory>(factory_, "RecordRebotTrajectory", context_);
  }
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(RecordRebotTrajectoryTest, RecordsEveryTrajectoryAndRefusesUnwritableAudit)
{
  auto tree = factory_.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Audit">
    <Action ID="RecordRebotTrajectory" trajectory="{trajectory}" file_path="{path}" />
    </BehaviorTree></root>)");
  const auto path = std::filesystem::temp_directory_path() / "rebot-record-trajectory-test.yaml";
  std::filesystem::remove(path);
  tree.rootBlackboard()->set("trajectory", plannedTrajectory());
  tree.rootBlackboard()->set("path", path.string());
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto first_size = std::filesystem::file_size(path);
  tree.haltTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_GT(std::filesystem::file_size(path), first_size);
  std::ifstream file(path);
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  EXPECT_NE(text.find("dispatched_at:"), std::string::npos);
  EXPECT_NE(text.find("positions:"), std::string::npos);
  std::filesystem::remove(path);
  tree.haltTree();
  tree.rootBlackboard()->set("path", (path / "missing" / "audit.yaml").string());
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(RecordRebotTrajectoryTest, ReportsMissingAuditInputsAndEmptyTrajectory)
{
  for (const std::string scenario : { "missing trajectory", "missing path", "empty trajectory" })
  {
    SCOPED_TRACE(scenario);
    auto tree = factory_.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Audit">
      <Action ID="RecordRebotTrajectory" trajectory="{trajectory}" file_path="{path}" />
      </BehaviorTree></root>)");
    if (scenario != "missing trajectory")
      tree.rootBlackboard()->set(
          "trajectory", scenario == "empty trajectory" ? trajectory_msgs::msg::JointTrajectory{} : plannedTrajectory());
    if (scenario != "missing path")
      tree.rootBlackboard()->set("path", std::string("/unused-audit.yaml"));
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
    EXPECT_THAT(context_->logger->consumeErrorLogBuffer(),
                HasSubstr("Provide a trajectory with at least one point and a file_path."));
  }
}
}  // namespace
}  // namespace rebot_bench_behaviors

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
