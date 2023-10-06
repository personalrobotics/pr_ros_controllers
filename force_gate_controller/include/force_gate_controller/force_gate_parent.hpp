// Copyright 2020 PAL Robotics S.L.
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

#ifndef FORCE_GATE_CONTROLLER__FORCE_GATE_PARENT_HPP_
#define FORCE_GATE_CONTROLLER__FORCE_GATE_PARENT_HPP_

// #include <string>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "force_gate_controller/tolerances.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.h"

// auto-generated by generate_parameter_library
#include "force_gate_controller_parameters.hpp"

namespace force_gate_controller
{
/**
 * \brief Defines shared functions and attributes to force-gate a controller.
 */
class ForceGateParent
{
protected:
  controller_interface::CallbackReturn read_force_gate_parameters(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

  WrenchTolerances wrench_tolerances_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::WrenchStamped>> rt_wrench_stamped_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscriber_ = nullptr;

  // Parameters from ROS for force_gate_controller
  std::shared_ptr<ParamListener> force_gate_param_listener_;
  Params force_gate_params_;

  bool check_wrench_threshold(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const rclcpp::Time & time);
};

}  // namespace force_gate_controller

#endif  // FORCE_GATE_CONTROLLER__FORCE_GATE_PARENT_HPP_