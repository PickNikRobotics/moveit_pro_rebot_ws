// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <velocity_force_controller/velocity_force_controller.hpp>

#include "vcan_controller_fixture.hpp"

namespace rebot_vcan_test
{
PoseJog MakePoseJog(const std::string& params_file)
{
  auto controller = std::make_shared<Configured<moveit_pro_controllers::VelocityForceController>>(params_file);
  return { controller, controller.get() };
}
}  // namespace rebot_vcan_test
