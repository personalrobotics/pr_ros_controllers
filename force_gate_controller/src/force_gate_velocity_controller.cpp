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

// #include <string>

#include "controller_interface/controller_interface.hpp"
#include "force_gate_controller/force_gate_velocity_controller.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "rclcpp/logging.hpp"
// #include "rclcpp/parameter.hpp"

namespace force_gate_controller
{
ForceGateVelocityController::ForceGateVelocityController()
: velocity_controllers::JointGroupVelocityController()
{
}

controller_interface::CallbackReturn ForceGateVelocityController::read_parameters()
{
  auto ret = velocity_controllers::JointGroupVelocityController::read_parameters();
  if (ret != CallbackReturn::SUCCESS)
  {
      return ret;
  }

  return read_force_gate_parameters(get_node());
}

controller_interface::CallbackReturn ForceGateVelocityController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // When the controller is activated, get the most up-to-date wrench tolerances
  auto ret = read_force_gate_parameters(get_node());
  if (ret != CallbackReturn::SUCCESS)
  {
      return ret;
  }
  return velocity_controllers::JointGroupVelocityController::on_activate(previous_state);
}

controller_interface::return_type ForceGateVelocityController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (!check_wrench_threshold(get_node(), get_node()->now()))
    {
        // stop all joints
        for (auto & command_interface : command_interfaces_)
        {
          command_interface.set_value(0.0);
        }
        return controller_interface::return_type::ERROR;
    }
    
    return velocity_controllers::JointGroupVelocityController::update(time, period);
}

}  // namespace force_gate_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  force_gate_controller::ForceGateVelocityController, controller_interface::ControllerInterface)
