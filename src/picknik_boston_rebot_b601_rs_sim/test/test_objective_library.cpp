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
#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_test_utils/ros_test.hpp>
#include <moveit_studio_agent/objective_server/objective_server.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

namespace
{
namespace fs = std::filesystem;
const std::string sim_package = "picknik_boston_rebot_b601_rs_sim";
const std::string office_package = "picknik_boston_rebot_b601_rs_hw";

std::string readFile(const fs::path& path)
{
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("Cannot read " + path.string());
  return { std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>() };
}

// These overlays add loader/library groups. The Python loader tests separately
// check the complete merge semantics and final simulation-only configuration.
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

class TwinObjectiveLibrary : public moveit_pro::test_utils::RosTest
{
};

TEST_F(TwinObjectiveLibrary, NativeServerLoadsCompleteInheritedLibraryWithoutExecutingMotion)
{
  const char* selected = std::getenv("REBOT_TEST_CONFIG_PACKAGE");
  const std::string package = selected ? selected : sim_package;
  const auto urdf = urdf::parseURDF(readFile(TEST_URDF));
  ASSERT_TRUE(urdf);
  auto srdf = std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf->initString(*urdf, readFile(fs::path(ament_index_cpp::get_package_share_directory(office_package)) /
                                               "config/moveit/rebot.srdf")));
  auto node = std::make_shared<rclcpp::Node>("test_twin_objective_library");
  auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false);
  context->robot_model = std::make_shared<moveit_pro::base::RobotModel>(urdf, srdf);
  moveit_studio::objective_server::ObjectiveServer server(context);
  std::vector<std::string> loaders, paths;
  libraries(package, loaders, paths);
  const auto loaded = server.registerBehaviorLoaderPlugins(loaders);
  ASSERT_TRUE(loaded.has_value()) << loaded.error().what;
  const auto registered = server.registerObjectives(paths);
  ASSERT_TRUE(registered.has_value()) << registered.error();
  for (const auto& error : registered->errors)
    ADD_FAILURE() << error;

  // Resolve filename overrides in the same parent-to-child order. Check the
  // complete workspace library, including inherited HOME and recorder trees.
  std::map<std::string, fs::path> effective;
  for (const auto& path : paths)
  {
    if (path.find("moveit_pro_objectives") != std::string::npos)
      continue;
    for (const auto& entry : fs::directory_iterator(path))
      if (entry.path().extension() == ".xml")
        effective[entry.path().filename().string()] = entry.path();
  }
  ASSERT_GE(effective.size(), 40u);
  for (const auto& [filename, path] : effective)
  {
    SCOPED_TRACE(filename);
    const auto text = readFile(path);
    const auto valid = server.verifyXML(text);
    ASSERT_TRUE(valid.has_value()) << valid.error().what;
    const auto name = moveit_studio::objective_server::utils::getObjectiveNameFromXml(text);
    ASSERT_TRUE(name.has_value()) << name.error();
    const auto created = server.createTree(name.value());
    EXPECT_TRUE(created.has_value()) << created.error().what;
  }

  // Only the three simulation refusal overrides are ticked. A factory with
  // built-ins alone cannot instantiate a motion, gain-service or recorder leaf.
  for (const auto& filename : { "start_gravity_comp.xml", "stop_gravity_comp.xml", "collect_rebot_gravity_data.xml" })
  {
    SCOPED_TRACE(filename);
    ASSERT_EQ(effective.at(filename).parent_path(),
              fs::path(ament_index_cpp::get_package_share_directory(sim_package)) / "objectives");
    BT::BehaviorTreeFactory factory;
    auto tree = factory.createTreeFromFile(effective.at(filename).string());
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  }
}
}  // namespace
