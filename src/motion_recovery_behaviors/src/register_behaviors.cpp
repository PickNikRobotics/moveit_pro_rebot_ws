// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#include <motion_recovery_behaviors/anchor_trajectory_to_command.hpp>
#include <motion_recovery_behaviors/clamp_robot_state_to_bounds.hpp>
#include <motion_recovery_behaviors/plan_to_joint_goal_from_bounded_start.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace motion_recovery_behaviors
{
class MotionRecoveryBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context) override
  {
    moveit_pro::behaviors::registerBehavior<ClampRobotStateToBounds>(factory, "ClampRobotStateToBounds", context);
    moveit_pro::behaviors::registerBehavior<PlanToJointGoalFromBoundedStart>(factory, "PlanToJointGoalFromBoundedStart",
                                                                             context);
    moveit_pro::behaviors::registerBehavior<AnchorTrajectoryToCommand>(factory, "AnchorTrajectoryToCommand", context);
  }
};
}  // namespace motion_recovery_behaviors
PLUGINLIB_EXPORT_CLASS(motion_recovery_behaviors::MotionRecoveryBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase)
