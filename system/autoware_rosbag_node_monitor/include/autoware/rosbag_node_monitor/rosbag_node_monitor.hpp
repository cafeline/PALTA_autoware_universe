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

#ifndef AUTOWARE__ROSBAG_NODE_MONITOR__ROSBAG_NODE_MONITOR_HPP_
#define AUTOWARE__ROSBAG_NODE_MONITOR__ROSBAG_NODE_MONITOR_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::rosbag_node_monitor
{

class RosbagNodeMonitor : public rclcpp::Node
{
public:
  explicit RosbagNodeMonitor(const rclcpp::NodeOptions & node_options);

private:
  void on_timer();
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  bool has_required_node(const std::string & required_node_name, std::string * matched_name);
  static std::string make_fully_qualified_name(
    const std::string & node_name, const std::string & node_namespace);
  static std::string remove_leading_slash(const std::string & name);

  diagnostic_updater::Updater updater_{this};
  rclcpp::TimerBase::SharedPtr timer_;
  double update_rate_;
};

}  // namespace autoware::rosbag_node_monitor

#endif  // AUTOWARE__ROSBAG_NODE_MONITOR__ROSBAG_NODE_MONITOR_HPP_
