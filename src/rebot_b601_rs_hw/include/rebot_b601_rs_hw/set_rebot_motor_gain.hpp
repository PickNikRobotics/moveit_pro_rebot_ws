// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <moveit_pro_behavior_interface/async_behavior_base.hpp>
#include <moveit_pro_behavior_interface/service_client_interface.hpp>
#include <optional>
#include <robstride_interfaces/srv/set_data_to_robstride.hpp>

namespace rebot_b601_rs_hw
{
/** Write one motor's motion-mode kp or kd through the owning driver's service.
 * Only reBot arm IDs 1-6 and the jaw ID 7 are accepted; arbitrary register writes are excluded.
 * Without a value it writes the configured gain (waiting at most 2 s): the first /robot_description message's single
 * RobStride ros2_control block, the GPIO whose ID param is motor_id, its kp or kd param. A description with no
 * RobStride block (MuJoCo twin, mock) has no motor gains, so nothing is written and the Behavior succeeds.
 * It waits at most 3 s for the service and 0.5 s for a response; halt cancels the wait. Success
 * acknowledges queuing in the driver, not motor readback. Halt/timeout cannot undo a request that has
 * already reached the driver.
 */
class SetRebotMotorGain final : public moveit_pro::behaviors::AsyncBehaviorBase
{
public:
  using Service = robstride_interfaces::srv::SetDataToRobstride;
  SetRebotMotorGain(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context);
  SetRebotMotorGain(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context,
                    std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<Service>> client);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

private:
  /** No request (without an error) means the description configures no RobStride motors. */
  tl::expected<std::optional<Service::Request>, std::string> createRequest();
  tl::expected<bool, std::string> doWork() override;
  tl::expected<void, std::string> doHalt() override;
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }
  std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<Service>> client_;
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace rebot_b601_rs_hw
