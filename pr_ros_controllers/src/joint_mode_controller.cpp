/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Personal Robotics Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/// \author Ethan Kroll Gordon

#include <pr_ros_controllers/joint_mode_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace pr_ros_controllers
{

namespace internal
{
// TODO: create a utils file?
/**
 * \return The map between \p t1 indices (implicitly encoded in return vector indices) to \t2 indices.
 * If \p t1 is <tt>"{C, B}"</tt> and \p t2 is <tt>"{A, B, C, D}"</tt>, the associated mapping vector is
 * <tt>"{2, 1}"</tt>.
 */
template <class T>
inline std::vector<unsigned int> mapping(const T& t1, const T& t2)
{
  typedef unsigned int SizeType;

  // t1 must be a subset of t2
  if (t1.size() > t2.size()) {return std::vector<SizeType>();}

  std::vector<SizeType> mapping_vector(t1.size()); // Return value
  for (typename T::const_iterator t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    typename T::const_iterator t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it) {return std::vector<SizeType>();}
    else
    {
      const SizeType t1_dist = std::distance(t1.begin(), t1_it);
      const SizeType t2_dist = std::distance(t2.begin(), t2_it);
      mapping_vector[t1_dist] = t2_dist;
    }
  }
  return mapping_vector;
}

inline hardware_interface::JointCommandModes modeFromString(std::string str) {
  if(str == "BEGIN")
    return hardware_interface::JointCommandModes::BEGIN;
  if(str == "POSITION")
    return hardware_interface::JointCommandModes::MODE_POSITION;
  if(str == "VELOCITY")
      return hardware_interface::JointCommandModes::MODE_VELOCITY;
  if(str == "EFFORT")
    return hardware_interface::JointCommandModes::MODE_EFFORT;
  if(str == "NOMODE" || str == "OTHER")
    return hardware_interface::JointCommandModes::NOMODE;
  if(str == "EMERGENCY_STOP" || str == "ESTOP")
    return hardware_interface::JointCommandModes::EMERGENCY_STOP;
  if(str == "SWITCHING")
      return hardware_interface::JointCommandModes::SWITCHING;
  
  // Else
  ROS_WARN_STREAM("Setting unknown mode '" << str.c_str() << "' to ERROR.");
  return hardware_interface::JointCommandModes::ERROR;
}

inline hardware_interface::JointCommandModes modeFromInt(int i) {
  switch(i) {
    case -1:
      return hardware_interface::JointCommandModes::BEGIN;
    case 0:
      return hardware_interface::JointCommandModes::MODE_POSITION;
    case 1:
      return hardware_interface::JointCommandModes::MODE_VELOCITY;
    case 2:
      return hardware_interface::JointCommandModes::MODE_EFFORT;
    case 3:
      return hardware_interface::JointCommandModes::NOMODE;
    case 4:
      return hardware_interface::JointCommandModes::EMERGENCY_STOP;
    case 5:
      return hardware_interface::JointCommandModes::SWITCHING;
    default:
      ROS_WARN_STREAM("Setting unknown mode '" << i << "' to ERROR.");
      return hardware_interface::JointCommandModes::ERROR;
  }
}

inline std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

} // namespace internal

bool JointModeController::init(hardware_interface::JointModeInterface* hw,
                                ros::NodeHandle&   n)
{

    // Cache controller node handle
    controller_nh_ = n;

    // Controller name
    name_ = internal::getLeafNamespace(controller_nh_);

    // Action status checking update rate
    double action_monitor_rate = 20.0;
    controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
    action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
    ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");

    // Initialize controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }
    
    // Clear joints_ first in case this is called twice
    joint_modes_.clear();
    for(unsigned int i=0; i<n_joints_; i++)
    {
      try
      {
        joint_modes_.push_back(hw->getHandle(joint_names_[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }


    // Initialize default joint mode
    param_name = "default";
    std::string default_mode_str = "BEGIN";
    n.getParam(param_name, default_mode_str);

    commands_buffer_.writeFromNonRT(std::vector<hardware_interface::JointCommandModes>(n_joints_, internal::modeFromString(default_mode_str)));

    // ROS API: Action interface
    action_server_.reset(new ActionServer(controller_nh_, "joint_mode_command",
                                        boost::bind(&JointModeController::goalCB,   this, _1),
                                        boost::bind(&JointModeController::cancelCB, this, _1),
                                        false));
    action_server_->start();

    return true;
}

void JointModeController::goalCB(GoalHandle gh)
{
  ROS_DEBUG_STREAM_NAMED(name_,"Received new action goal");
  pr_control_msgs::JointModeCommandResult result;
  std::vector<int> command = gh.getGoal()->modes;

  // Preconditions
  if (!this->isRunning())
  {
    result.message = "Can't accept new action goals. Controller is not running.";
    ROS_ERROR_STREAM_NAMED(name_, result.message);
    result.success = false;
    gh.setRejected(result);
    return;
  }


  if (gh.getGoal()->joint_names.size() != command.size()) {
    result.message = "Size of command must match size of joint_names.";
    ROS_ERROR_STREAM_NAMED(name_, result.message);
    result.success = false;
    gh.setRejected(result);
    return;
  }

  // Goal should specify valid controller joints (they can be ordered differently). Reject if this is not the case
  using internal::mapping;
  std::vector<unsigned int> mapping_vector = mapping(gh.getGoal()->joint_names, joint_names_);

  if (mapping_vector.size() != gh.getGoal()->joint_names.size())
  {
    result.message = "Joints on incoming goal don't match the controller joints.";
    ROS_ERROR_STREAM_NAMED(name_, result.message);
    result.success = false;
    gh.setRejected(result);
    return;
  }

  // Accept new goal
  gh.setAccepted();

  // update new command
  std::vector< hardware_interface::JointCommandModes > new_commands(*commands_buffer_.readFromNonRT());
  for(int i = 0; i < mapping_vector.size(); i++) {
    new_commands[mapping_vector[i]] = internal::modeFromInt(command[i]);
  }
  commands_buffer_.writeFromNonRT(new_commands);

  // Send successful result
  result.message = "Success";
  result.success = true;
  gh.setSucceeded(result);
}

} // namespace joint_mode_controller

PLUGINLIB_EXPORT_CLASS(pr_ros_controllers::JointModeController,controller_interface::ControllerBase)
