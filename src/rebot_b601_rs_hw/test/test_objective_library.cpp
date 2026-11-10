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
#include <map>
#include <sstream>
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
  std::ostringstream text;
  text << input.rdbuf();
  return text.str();
}

// Match keyed library/group overrides (including an empty plugin list), not
// append-only inheritance. The Python contract test uses the real loader.
void collectLibraries(const std::string& package, std::map<std::string, std::vector<std::string>>& loaders,
                      std::vector<std::pair<std::string, std::string>>& directories)
{
  const auto share = fs::path(ament_index_cpp::get_package_share_directory(package));
  const auto config = YAML::LoadFile((share / "config/config.yaml").string());
  if (config["based_on_package"])
    collectLibraries(config["based_on_package"].as<std::string>(), loaders, directories);
  const auto objectives = config["objectives"];
  for (const auto& group : objectives["behavior_loader_plugins"])
  {
    auto& group_loaders = loaders[group.first.as<std::string>()];
    group_loaders.clear();
    for (const auto& loader : group.second)
      group_loaders.push_back(loader.as<std::string>());
  }
  for (const auto& library : objectives["objective_library_paths"])
  {
    const auto id = library.first.as<std::string>();
    std::erase_if(directories, [&](const auto& entry) { return entry.first == id; });
    const auto entry = library.second;
    const auto library_share = ament_index_cpp::get_package_share_directory(entry["package_name"].as<std::string>());
    directories.emplace_back(id, (fs::path(library_share) / entry["relative_path"].as<std::string>()).string());
  }
}

class ObjectiveLibraryTest : public moveit_pro::test_utils::RosTest, public testing::WithParamInterface<std::string>
{
};

TEST_P(ObjectiveLibraryTest, NativeServerLoadsAndInstantiatesStockObjectivesWithoutExecuting)
{
  const auto mock_share = fs::path(ament_index_cpp::get_package_share_directory("rebot_b601_rs_mock"));
  const auto urdf = urdf::parseURDF(readFile(TEST_URDF));
  ASSERT_TRUE(urdf);
  auto srdf = std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf->initString(*urdf, readFile(mock_share / "config/moveit/rebot.srdf")));
  auto node = std::make_shared<rclcpp::Node>("test_rebot_objective_library");
  auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false);
  context->robot_model = std::make_shared<moveit_pro::base::RobotModel>(urdf, srdf);
  moveit_studio::objective_server::ObjectiveServer server(context);
  std::map<std::string, std::vector<std::string>> loader_groups;
  std::vector<std::pair<std::string, std::string>> library_specs;
  collectLibraries(GetParam(), loader_groups, library_specs);
  std::vector<std::string> loaders, directories;
  for (const auto& [group, names] : loader_groups)
    loaders.insert(loaders.end(), names.begin(), names.end());
  for (const auto& [id, directory] : library_specs)
    directories.push_back(directory);
  const auto loaded = server.registerBehaviorLoaderPlugins(loaders);
  ASSERT_TRUE(loaded.has_value()) << loaded.error().what;
  const auto registered = server.registerObjectives(directories);
  ASSERT_TRUE(registered.has_value()) << registered.error();
  for (const auto& error : registered->errors)
    ADD_FAILURE() << error;

  // Canonical full validation against this server's loaded plugin manifest,
  // followed by real tree instantiation to check ports and inherited SubTrees.
  // Never tick: the physical config's callbacks cannot send gains or motion.
  std::vector<fs::path> owned_libraries{ mock_share / "objectives" };
  if (GetParam() != "rebot_b601_rs_mock")
    owned_libraries.emplace_back(fs::path(ament_index_cpp::get_package_share_directory(GetParam())) / "objectives");
  if (GetParam() == "rebot_b601_rs_hw")
    EXPECT_FALSE(server.createTree("Reset MuJoCo Sim").has_value());
  for (const auto& library : owned_libraries)
    for (const auto& entry : fs::directory_iterator(library))
    {
      if (entry.path().extension() != ".xml")
        continue;
      SCOPED_TRACE(entry.path().string());
      const auto text = readFile(entry.path());
      const auto valid = server.verifyXML(text);
      ASSERT_TRUE(valid.has_value()) << valid.error().what;
      const auto name = moveit_studio::objective_server::utils::getObjectiveNameFromXml(text);
      ASSERT_TRUE(name.has_value()) << name.error();
      const auto created = server.createTree(name.value());
      EXPECT_TRUE(created.has_value()) << created.error().what;
    }
}

INSTANTIATE_TEST_SUITE_P(StockBackends, ObjectiveLibraryTest,
                         testing::Values("rebot_b601_rs_mock", "rebot_b601_rs_sim", "rebot_b601_rs_hw"));
}  // namespace
