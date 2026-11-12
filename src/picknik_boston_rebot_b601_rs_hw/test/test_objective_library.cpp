// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_test_utils/ros_test.hpp>
#include <moveit_studio_agent/objective_server/objective_server.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace
{
namespace fs = std::filesystem;
std::string readFile(const fs::path& path)
{
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("Cannot read " + path.string());
  return { std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>() };
}

void libraries(const std::string& package, std::vector<std::string>& loaders, std::vector<std::string>& paths)
{
  const auto share = fs::path(ament_index_cpp::get_package_share_directory(package));
  const auto config = YAML::LoadFile((share / "config/config.yaml").string());
  if (config["based_on_package"])
    libraries(config["based_on_package"].as<std::string>(), loaders, paths);
  for (const auto& group : config["objectives"]["behavior_loader_plugins"])
    for (const auto& loader : group.second)
      loaders.push_back(loader.as<std::string>());
  for (const auto& library : config["objectives"]["objective_library_paths"])
  {
    const auto entry = library.second;
    paths.push_back((fs::path(ament_index_cpp::get_package_share_directory(entry["package_name"].as<std::string>())) /
                     entry["relative_path"].as<std::string>())
                        .string());
  }
}

class BostonObjectiveLibrary : public moveit_pro::test_utils::RosTest
{
};

TEST_F(BostonObjectiveLibrary, NativeServerLoadsOfficeAndSharedHelpersWithoutDuplicateIdsOrExecution)
{
  const auto urdf = urdf::parseURDF(readFile(TEST_URDF));
  ASSERT_TRUE(urdf);
  auto srdf = std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf->initString(*urdf, readFile(TEST_SRDF)));
  auto node = std::make_shared<rclcpp::Node>("test_boston_objective_library");
  auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false);
  context->robot_model = std::make_shared<moveit_pro::base::RobotModel>(urdf, srdf);
  moveit_studio::objective_server::ObjectiveServer server(context);
  std::vector<std::string> loaders, paths;
  libraries("picknik_boston_rebot_b601_rs_hw", loaders, paths);
  const auto loaded = server.registerBehaviorLoaderPlugins(loaders);
  ASSERT_TRUE(loaded.has_value()) << loaded.error().what;
  const auto registered = server.registerObjectives(paths);
  ASSERT_TRUE(registered.has_value()) << registered.error();
  for (const auto& error : registered->errors)
    ADD_FAILURE() << error;
  // These trees are instantiated, never ticked: no camera, driver, recording or motion.
  for (const auto& entry : fs::directory_iterator(PACKAGE_DIR "/objectives"))
  {
    if (entry.path().extension() != ".xml")
      continue;
    SCOPED_TRACE(entry.path().string());
    const auto text = readFile(entry.path());
    const auto valid = server.verifyXML(text);
    EXPECT_TRUE(valid.has_value()) << valid.error().what;
    const auto name = moveit_studio::objective_server::utils::getObjectiveNameFromXml(text);
    ASSERT_TRUE(name.has_value()) << name.error();
    const auto created = server.createTree(name.value());
    EXPECT_TRUE(created.has_value()) << created.error().what;
  }
  // The inherited recovery resolves its office waypoint override and recorder.
  const auto home = server.createTree("Return reBot Leader Home");
  EXPECT_TRUE(home.has_value()) << home.error().what;
}
}  // namespace
