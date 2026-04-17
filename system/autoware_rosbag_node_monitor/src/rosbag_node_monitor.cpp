// Copyright 2026 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/rosbag_node_monitor/rosbag_node_monitor.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <functional>
#include <string>

namespace autoware::rosbag_node_monitor
{

RosbagNodeMonitor::RosbagNodeMonitor(const rclcpp::NodeOptions & node_options)
: Node("rosbag_node_monitor", node_options)
{
  update_rate_ = declare_parameter<double>("update_rate", 2.0);
  declare_parameter<bool>("enabled", false);
  declare_parameter<std::string>("required_node_name", "/rosbag2_recorder");

  updater_.setHardwareID("rosbag_node_monitor");
  updater_.add("rosbag_node_alive", this, &RosbagNodeMonitor::produce_diagnostics);

  const auto period_ns = rclcpp::Rate(update_rate_).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&RosbagNodeMonitor::on_timer, this));
}

void RosbagNodeMonitor::on_timer()
{
  updater_.force_update();
}

void RosbagNodeMonitor::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  const auto enabled = get_parameter("enabled").as_bool();
  const auto required_node_name = get_parameter("required_node_name").as_string();

  stat.add("enabled", enabled ? "true" : "false");
  stat.add("required_node_name", required_node_name);

  if (!enabled) {
    stat.summary(DiagnosticStatus::OK, "rosbag AUTO gate is disabled");
    return;
  }

  if (required_node_name.empty()) {
    stat.summary(DiagnosticStatus::ERROR, "required_node_name is empty");
    return;
  }

  std::string matched_name;
  if (has_required_node(required_node_name, &matched_name)) {
    stat.add("matched_node_name", matched_name);
    stat.summary(DiagnosticStatus::OK, "required rosbag node is running");
    return;
  }

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
    "Required rosbag node '%s' is not running.", required_node_name.c_str());
  stat.summary(DiagnosticStatus::ERROR, "required rosbag node is not running");
}

bool RosbagNodeMonitor::has_required_node(
  const std::string & required_node_name, std::string * matched_name)
{
  const auto is_fully_qualified = !required_node_name.empty() && required_node_name.front() == '/';
  const auto required_without_leading_slash = remove_leading_slash(required_node_name);

  for (const auto & [node_name, node_namespace] :
       get_node_graph_interface()->get_node_names_and_namespaces()) {
    const auto full_name = make_fully_qualified_name(node_name, node_namespace);

    if (is_fully_qualified) {
      if (full_name == required_node_name) {
        if (matched_name) *matched_name = full_name;
        return true;
      }
      continue;
    }

    if (
      node_name == required_node_name ||
      remove_leading_slash(full_name) == required_without_leading_slash) {
      if (matched_name) *matched_name = full_name;
      return true;
    }
  }
  return false;
}

std::string RosbagNodeMonitor::make_fully_qualified_name(
  const std::string & node_name, const std::string & node_namespace)
{
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + node_name;
  }
  if (node_namespace.back() == '/') {
    return node_namespace + node_name;
  }
  return node_namespace + "/" + node_name;
}

std::string RosbagNodeMonitor::remove_leading_slash(const std::string & name)
{
  if (!name.empty() && name.front() == '/') {
    return name.substr(1);
  }
  return name;
}

}  // namespace autoware::rosbag_node_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::rosbag_node_monitor::RosbagNodeMonitor)
