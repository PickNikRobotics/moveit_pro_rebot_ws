// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rebot_b601_rs_hw/call_rebot_leader_service.hpp>
#include <rebot_b601_rs_hw/set_rebot_motor_gain.hpp>

namespace rebot_b601_rs_hw
{
class RebotHardwareBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context) override
  {
    moveit_pro::behaviors::registerBehavior<SetRebotMotorGain>(factory, "SetRebotMotorGain", context);
  }
};

/** Leader-follower Behaviors; only configs that enable the optional leader load this. */
class RebotLeaderBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context) override
  {
    moveit_pro::behaviors::registerBehavior<CallRebotLeaderService>(factory, "CallRebotLeaderService", context);
  }
};
}  // namespace rebot_b601_rs_hw

PLUGINLIB_EXPORT_CLASS(rebot_b601_rs_hw::RebotHardwareBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase)
PLUGINLIB_EXPORT_CLASS(rebot_b601_rs_hw::RebotLeaderBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase)
