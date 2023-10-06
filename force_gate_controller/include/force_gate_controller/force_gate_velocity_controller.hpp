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

#ifndef FORCE_GATE_CONTROLLER__FORCE_GATE_VELOCITY_CONTROLLER_HPP_
#define FORCE_GATE_CONTROLLER__FORCE_GATE_VELOCITY_CONTROLLER_HPP_

// #include <string>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "velocity_controllers/joint_group_velocity_controller.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "force_gate_controller/force_gate_parent.hpp"
#include "force_gate_controller/tolerances.hpp"
#include "force_gate_controller/visibility_control.h"

namespace force_gate_controller
{
/**
 * \brief Forward command controller for a set of velocity controlled joints (linear or angular).
 *
 * This class forwards the commanded velocities down to a set of joints.
 *
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::msg::Float64MultiArray) : The velocity commands to apply.
 */
class ForceGateVelocityController : public velocity_controllers::JointGroupVelocityController, public ForceGateParent
{
public:
  FORCE_GATE_CONTROLLER_PUBLIC
  ForceGateVelocityController();

  FORCE_GATE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  FORCE_GATE_CONTROLLER_PUBLIC
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // Overridden functions to read parameters from ROS.
  controller_interface::CallbackReturn read_parameters() override;
};

}  // namespace force_gate_controller

#endif  // FORCE_GATE_CONTROLLER__FORCE_GATE_VELOCITY_CONTROLLER_HPP_
