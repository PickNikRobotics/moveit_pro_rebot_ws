// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#include <rebot_b601_rs_hw/set_rebot_motor_gain.hpp>

#include <tinyxml2.h>
#include <cmath>
#include <future>
#include <memory>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

namespace rebot_b601_rs_hw
{
namespace
{
constexpr auto kServiceName = "/robstride_hardware_interface/set_data_to_robstride";
constexpr std::chrono::seconds kDescriptionTimeout{ 2 };

tl::expected<std::string, std::string> receiveRobotDescription(const moveit_pro::behaviors::BehaviorContext& context)
{
  auto received = std::make_shared<std::promise<std::string>>();
  auto once = std::make_shared<std::once_flag>();
  auto future = received->get_future();
  rclcpp::SubscriptionOptions options;
  options.callback_group = context.reentrant_callback_group;
  const auto subscription = context.node->create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).reliable().transient_local(),
      [received, once](const std_msgs::msg::String::ConstSharedPtr message) {
        std::call_once(*once, [&] { received->set_value(message->data); });
      },
      options);
  if (future.wait_for(kDescriptionTimeout) != std::future_status::ready)
    return tl::make_unexpected("No /robot_description received within 2 s, so the configured gain is unknown.");
  return future.get();
}

std::string text(const tinyxml2::XMLElement* element)
{
  return element != nullptr && element->GetText() != nullptr ? element->GetText() : "";
}

const tinyxml2::XMLElement* param(const tinyxml2::XMLElement& parent, const std::string& name)
{
  for (auto* element = parent.FirstChildElement("param"); element != nullptr;
       element = element->NextSiblingElement("param"))
    if (element->Attribute("name", name.c_str()) != nullptr)
      return element;
  return nullptr;
}

/** The initial driver configuration: the one RobStride block's GPIO whose ID param is the motor ID.
 * No value means the description has no RobStride block, so no motor gain is configured. */
tl::expected<std::optional<double>, std::string> configuredGain(const std::string& description, int id,
                                                                const std::string& gain)
{
  tinyxml2::XMLDocument document;
  if (document.Parse(description.c_str()) != tinyxml2::XML_SUCCESS || document.RootElement() == nullptr)
    return tl::make_unexpected("/robot_description is not valid XML.");
  std::vector<const tinyxml2::XMLElement*> controls;
  for (auto* control = document.RootElement()->FirstChildElement("ros2_control"); control != nullptr;
       control = control->NextSiblingElement("ros2_control"))
  {
    const auto* hardware = control->FirstChildElement("hardware");
    if (hardware != nullptr &&
        text(hardware->FirstChildElement("plugin")) == "robstride_hardware_interface/RobstrideHardware")
      controls.push_back(control);
  }
  if (controls.empty())
    return std::nullopt;
  if (controls.size() != 1)
    return tl::make_unexpected("/robot_description must have exactly one RobStride ros2_control block.");
  std::vector<const tinyxml2::XMLElement*> gpios;
  for (auto* gpio = controls.front()->FirstChildElement("gpio"); gpio != nullptr;
       gpio = gpio->NextSiblingElement("gpio"))
    if (text(param(*gpio, "ID")) == std::to_string(id))
      gpios.push_back(gpio);
  double value = 0.0;
  if (gpios.size() != 1 || param(*gpios.front(), gain) == nullptr ||
      param(*gpios.front(), gain)->QueryDoubleText(&value) != tinyxml2::XML_SUCCESS)
    return tl::make_unexpected("/robot_description has no single configured " + gain + " for motor " +
                               std::to_string(id) + ".");
  return value;
}
}  // namespace

SetRebotMotorGain::SetRebotMotorGain(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context)
  : SetRebotMotorGain(name, config, context,
                      std::make_unique<moveit_pro::behaviors::RclcppClientInterface<Service>>(context))
{
}

SetRebotMotorGain::SetRebotMotorGain(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context,
                                     std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<Service>> client)
  : AsyncBehaviorBase(name, config, context), client_(std::move(client))
{
}

BT::PortsList SetRebotMotorGain::providedPorts()
{
  return { BT::InputPort<int>("motor_id", "Motor ID: arm 1-6 or jaw 7."),
           BT::InputPort<std::string>("gain", "Motion-mode gain: kp or kd only."),
           BT::InputPort<double>("value", "Nonnegative gain: kp in Nm/rad, kd in Nm s/rad. Must fit the motor's "
                                          "protocol range. Omit it to write the motor's configured gain from "
                                          "/robot_description.") };
}

BT::KeyValueVector SetRebotMotorGain::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Queues one arm or jaw motor kp/kd update through the existing RobStride driver, either the given value "
             "or the motor's configured gain from /robot_description (nothing on a description without RobStride "
             "hardware). Requires motion mode. "
             "Re-anchor the trajectory controller to the measured pose before restoring kp. "
             "Success is a driver acknowledgement, not motor readback; cancellation does not roll back writes." } };
}

tl::expected<bool, std::string> SetRebotMotorGain::doWork()
{
  client_->initialize(kServiceName, std::chrono::duration<double>{ 3.0 }, std::chrono::duration<double>{ 0.5 });
  if (!client_->waitForServiceServer())
    return tl::make_unexpected(std::string("Server not available for service ") + kServiceName);
  notifyCanHalt();
  const auto request = createRequest();
  if (!request)
    return tl::make_unexpected("Failed to create service request: " + request.error());
  if (!request.value())
  {
    getBehaviorContext()->logger->publishInfoMessage(
        name(), "/robot_description has no RobStride hardware, so there is no motor gain to restore.");
    return true;
  }
  const auto response = client_->syncSendRequest(*request.value());
  if (!response)
    return tl::make_unexpected(std::string("Failed to send request to service server at \"") + kServiceName +
                               "\": " + response.error());
  if (!response.value().result)
    return tl::make_unexpected(
        "RobStride driver rejected the motor gain write. Earlier writes, if any, remain applied.");
  return true;
}

tl::expected<void, std::string> SetRebotMotorGain::doHalt()
{
  client_->cancelRequest();
  return {};
}

tl::expected<std::optional<SetRebotMotorGain::Service::Request>, std::string> SetRebotMotorGain::createRequest()
{
  const auto inputs =
      moveit_pro::behaviors::getRequiredInputs(getInput<int>("motor_id"), getInput<std::string>("gain"));
  if (!inputs)
    return tl::make_unexpected("Provide motor_id and gain: " + inputs.error());
  const auto& [id, gain] = inputs.value();
  if (id < 1 || id > 7)
    return tl::make_unexpected("motor_id must be an arm motor ID from 1 to 6 or the jaw motor ID 7.");
  if (gain != "kp" && gain != "kd")
    return tl::make_unexpected("gain must be kp or kd; arbitrary register writes are not supported.");
  double value = 0.0;
  if (config().input_ports.contains("value"))
  {
    const auto given = getInput<double>("value");
    if (!given)
      return tl::make_unexpected("Provide a valid value: " + given.error());
    value = given.value();
  }
  else
  {
    const auto description = receiveRobotDescription(*getBehaviorContext());
    if (!description)
      return tl::make_unexpected(description.error());
    const auto configured = configuredGain(description.value(), id, gain);
    if (!configured)
      return tl::make_unexpected(configured.error());
    if (!configured.value())
      return std::nullopt;
    value = *configured.value();
  }
  // Type-1 encoding ranges from robstride_sdk/robstride_protocol.hpp, not torque clamps:
  // arm IDs 1-3 are RS06, 4-6 and the jaw (7) are RS00. Reject rather than silently saturate the gain.
  const double maximum = gain == "kp" ? (id <= 3 ? 5000.0 : 500.0) : (id <= 3 ? 100.0 : 5.0);
  if (!std::isfinite(value) || value < 0.0 || value > maximum)
    return tl::make_unexpected("value must be finite and in [0, " + std::to_string(maximum) + "] for this motor/gain.");
  Service::Request request;
  request.header.stamp = getBehaviorContext()->node->now();
  request.id = static_cast<std::uint8_t>(id);
  request.item_name = gain;
  request.item_data = static_cast<float>(value);
  return request;
}
}  // namespace rebot_b601_rs_hw
