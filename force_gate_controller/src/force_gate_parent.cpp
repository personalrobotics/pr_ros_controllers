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
// #include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "force_gate_controller/force_gate_parent.hpp"
#include "rclcpp/logging.hpp"
// #include "rclcpp/parameter.hpp"

namespace force_gate_controller
{

controller_interface::CallbackReturn ForceGateParent::read_force_gate_parameters(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
  if (!force_gate_param_listener_)
  {
    force_gate_param_listener_ = std::make_shared<ParamListener>(node);
    if (!force_gate_param_listener_) {
      RCLCPP_ERROR(node->get_logger(), "Error encountered during init");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  force_gate_params_ = force_gate_param_listener_->get_params();

  // Update wrench tolerances if thresholding enabled
  rt_wrench_stamped_.writeFromNonRT(nullptr);
  if (force_gate_params_.wrench_threshold.topic != "")
  {
    RCLCPP_INFO(node->get_logger(), "Updating Wrench Thresholds");

    wrench_tolerances_.timeout = rclcpp::Duration::from_seconds(force_gate_params_.wrench_threshold.timeout);
    wrench_tolerances_.forceTotal = force_gate_params_.wrench_threshold.fMag;
    wrench_tolerances_.forceVec[0] = force_gate_params_.wrench_threshold.fx;
    wrench_tolerances_.forceVec[1] = force_gate_params_.wrench_threshold.fy;
    wrench_tolerances_.forceVec[2] = force_gate_params_.wrench_threshold.fz;
    wrench_tolerances_.torqueTotal = force_gate_params_.wrench_threshold.tMag;
    wrench_tolerances_.torqueVec[0] = force_gate_params_.wrench_threshold.tx;
    wrench_tolerances_.torqueVec[1] = force_gate_params_.wrench_threshold.ty;
    wrench_tolerances_.torqueVec[2] = force_gate_params_.wrench_threshold.tz;

    // Subscribe to wrench topic
    wrench_subscriber_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      force_gate_params_.wrench_threshold.topic, rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { 
        rt_wrench_stamped_.writeFromNonRT(msg);
    });
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool ForceGateParent::check_wrench_threshold(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const rclcpp::Time & time)
{
  // Check if enabled
  if (force_gate_params_.wrench_threshold.topic == "")
  {
    return true;
  }

  // Read stamped wrench
  auto wrench_stamped = *rt_wrench_stamped_.readFromRT();
  if (!wrench_stamped) {
    RCLCPP_WARN(node->get_logger(), "WrenchStamped not received.");
    return false;
  }
  
  // Check all relevant tolerances
  if (wrench_tolerances_.timeout != rclcpp::Duration(0, 0)) {
    if (time - wrench_tolerances_.timeout > wrench_stamped->header.stamp) {
      RCLCPP_WARN(node->get_logger(), "WrenchStamped timeout.");
      return false;
    }
  }

  double forceSumSq = 0.0;
  if (wrench_tolerances_.forceVec[0] != 0.0) {
    if (wrench_stamped->wrench.force.x > wrench_tolerances_.forceVec[0]) {
      RCLCPP_WARN(node->get_logger(), "Wrench: Fx violation.");
      return false;
    }
  }
  forceSumSq += wrench_stamped->wrench.force.x * wrench_stamped->wrench.force.x;
  if (wrench_tolerances_.forceVec[1] != 0.0) {
    if (wrench_stamped->wrench.force.y > wrench_tolerances_.forceVec[1]) {
      RCLCPP_WARN(node->get_logger(), "Wrench: Fy violation.");
      return false;
    }
  }
  forceSumSq += wrench_stamped->wrench.force.y * wrench_stamped->wrench.force.y;
  if (wrench_tolerances_.forceVec[2] != 0.0) {
    if (wrench_stamped->wrench.force.z > wrench_tolerances_.forceVec[2]) {
      RCLCPP_WARN(node->get_logger(), "Wrench: Fz violation.");
      return false;
    }
  }
  forceSumSq += wrench_stamped->wrench.force.z * wrench_stamped->wrench.force.z;
  if (wrench_tolerances_.forceTotal != 0.0) {
    if (forceSumSq > wrench_tolerances_.forceTotal*wrench_tolerances_.forceTotal) {
      RCLCPP_WARN(node->get_logger(), "Wrench: ||F|| violation.");
      return false;
    }
  }

  double torqueSumSq = 0.0;
  if (wrench_tolerances_.torqueVec[0] != 0.0) {
    if (wrench_stamped->wrench.torque.x > wrench_tolerances_.torqueVec[0]) {
      RCLCPP_WARN(node->get_logger(), "Wrench: Tx violation.");
      return false;
    }
  }
  torqueSumSq += wrench_stamped->wrench.torque.x * wrench_stamped->wrench.torque.x;
  if (wrench_tolerances_.torqueVec[1] != 0.0) {
    if (wrench_stamped->wrench.torque.y > wrench_tolerances_.torqueVec[1]) {
      RCLCPP_WARN(node->get_logger(), "Wrench: Ty violation.");
      return false;
    }
  }
  torqueSumSq += wrench_stamped->wrench.torque.y * wrench_stamped->wrench.torque.y;
  if (wrench_tolerances_.torqueVec[2] != 0.0) {
    if (wrench_stamped->wrench.torque.z > wrench_tolerances_.torqueVec[2]) {
      RCLCPP_WARN(node->get_logger(), "Wrench: Tz violation.");
      return false;
    }
  }
  torqueSumSq += wrench_stamped->wrench.torque.z * wrench_stamped->wrench.torque.z;
  if (wrench_tolerances_.torqueTotal != 0.0) {
    if (torqueSumSq > wrench_tolerances_.torqueTotal*wrench_tolerances_.torqueTotal) {
      RCLCPP_WARN(node->get_logger(), "Wrench: ||T|| violation.");
      return false;
    }
  }

  return true;
}

}  // namespace force_gate_controller
