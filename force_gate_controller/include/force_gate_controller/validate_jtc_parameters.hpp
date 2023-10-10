// Copyright (c) 2022 ros2_control Development Team
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

#ifndef FORCE_GATE_CONTROLLER__VALIDATE_JTC_PARAMETERS_HPP_
#define FORCE_GATE_CONTROLLER__VALIDATE_JTC_PARAMETERS_HPP_

#include <string>
#include <vector>

#include "rclcpp/parameter.hpp"
#include "rsl/algorithm.hpp"
#include "tl_expected/expected.hpp"

namespace force_gate_controller
{
tl::expected<void, std::string> command_interface_type_combinations(
  rclcpp::Parameter const & parameter);

tl::expected<void, std::string> state_interface_type_combinations(
  rclcpp::Parameter const & parameter);

}  // namespace force_gate_controller

#endif  // FORCE_GATE_CONTROLLER__VALIDATE_JTC_PARAMETERS_HPP_
