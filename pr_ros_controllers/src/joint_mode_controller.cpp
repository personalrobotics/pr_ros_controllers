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

bool ForwardJointGroupCommandController<HardwareInterface>::init(hardware_interface::JointModeInterface* jmi,
                                                                    ros::NodeHandle&   n)
{
    // Get Joint 
    if(jmt && n.getParam("mode_handle", handle_name)) {
      ROS_DEBUG_STREAM_NAMED(name_, "Enabling joint mode handling.");
      mode_handle_.reset(new hardware_interface::JointModeHandle(jmt->getHandle(handle_name)));
    }

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
    joints_.clear();
    for(unsigned int i=0; i<n_joints_; i++)
    {
      try
      {
        joints_.push_back(hw->getHandle(joint_names_[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }

    commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
    default_commands_.resize(n_joints_);

    // ROS API: Subscribed topics
    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &ForwardJointGroupCommandController::commandCB, this);

    // ROS API: Action interface
    action_server_.reset(new ActionServer(controller_nh_, "joint_group_command",
                                        boost::bind(&ForwardJointGroupCommandController::goalCB,   this, _1),
                                        boost::bind(&ForwardJointGroupCommandController::cancelCB, this, _1),
                                        false));
    action_server_->start();

    // Add Joint Mode switching if handle provided
    hardware_interface::JointModeInterface* jmt = robot_hw->get<hardware_interface::JointModeInterface>();
    std::string handle_name;
    if(jmt && n.getParam("mode_handle", handle_name)) {
      ROS_DEBUG_STREAM_NAMED(name_, "Enabling joint mode handling.");
      mode_handle_.reset(new hardware_interface::JointModeHandle(jmt->getHandle(handle_name)));
    }
    return true;
}

template <class T>
void forward_command_controller::ForwardJointGroupCommandController<T>::goalCB(GoalHandle gh)
{
  // Set as position command
  setGoal(gh, gh.getGoal()->command.velocities);
}

PLUGINLIB_EXPORT_CLASS(pr_ros_controllers::JointModeController,controller_interface::ControllerBase)
