// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
// Parse the installed workspace YAML with the actual pinned driver, no plugin
// construction, lifecycle transition, CAN socket or motor-control context.
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <robstride_hardware_interface/relaxation.hpp>

#include <string>
#include <vector>

TEST(RelaxSafePosesDriver, InstalledWorkspaceSchemaAndValuesMatchTheRealParser)
{
  const std::vector<std::string> joints{ "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7" };
  const auto poses = robstride_hardware_interface::LoadRelaxSafePoses(SAFE_POSES_FILE, joints);
  const auto entries = YAML::LoadFile(SAFE_POSES_FILE)["relax_safe_poses"];
  ASSERT_EQ(poses.size(), entries.size());
  ASSERT_FALSE(poses.empty());
  EXPECT_EQ(poses.front().name, "HOME");
  for (std::size_t p = 0; p < poses.size(); ++p)
  {
    SCOPED_TRACE(poses[p].name);
    EXPECT_EQ(poses[p].name, entries[p]["name"].as<std::string>());
    ASSERT_EQ(poses[p].position.size(), joints.size());
    ASSERT_EQ(poses[p].tolerance_rad.size(), joints.size());
    for (std::size_t j = 0; j < joints.size(); ++j)
    {
      const auto entry = entries[p]["joints"][joints[j]];
      EXPECT_DOUBLE_EQ(poses[p].position[j], entry["position_rad"].as<double>());
      EXPECT_DOUBLE_EQ(poses[p].tolerance_rad[j], entry["tolerance_rad"].as<double>());
    }
    const auto stillness = entries[p]["stillness"];
    EXPECT_DOUBLE_EQ(poses[p].window_s, stillness["window_s"].as<double>());
    EXPECT_DOUBLE_EQ(poses[p].max_span_rad, stillness["max_span_rad"].as<double>());
    EXPECT_DOUBLE_EQ(poses[p].max_abs_velocity_rad_s, stillness["max_abs_velocity_rad_s"].as<double>(0.01));
  }
}
