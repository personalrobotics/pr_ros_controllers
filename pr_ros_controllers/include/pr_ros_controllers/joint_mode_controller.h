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

#pragma once


#include <vector>
#include <string>

#include <ros/node_handle.h>
#include <realtime_tools/realtime_buffer.h>

// actionlib
#include <actionlib/server/action_server.h>

// ROS messages
#include <pr_control_msgs/JointModeCommandAction.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_mode_interface.h>


namespace pr_ros_controllers
{

/**
 * \brief Joint mode controller for a set of joints.
 *
 * This class provides an actionlib interface the control mode of the provided joints.
 *
 *
 * \section ROS interface
 *
 * \param joints Names of the joints to control.
 * \param default String of mode to initialize all joints.
 *                Possible values: BEGIN, POSITION, VELOCITY, EFFORT, NOMODE, OTHER, SWITCHING
 *                All other values will be converted to ERROR
 * \see hardware_interface::JointCommandModes
 *
 */

class JointModeController: public controller_interface::Controller<hardware_interface::JointModeInterface>
{
public:
  JointModeController() : controller_interface::Controller<hardware_interface::JointModeInterface>() {}
  ~JointModeController() {}

  bool init(hardware_interface::JointModeInterface* hw, ros::NodeHandle &n) override;

  void starting(const ros::Time& /*time*/) { /* Do Nothing */}
  void stopping(const ros::Time& /*time*/) { /* Do Nothing */}
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
  {
    std::vector<hardware_interface::JointCommandModes> & commands = *commands_buffer_.readFromRT();
    for(unsigned int i=0; i<n_joints_; i++)
    {  joint_modes_[i].setMode(commands[i]); }
  }

protected:
  typedef actionlib::ActionServer<pr_control_msgs::JointModeCommandAction>                    ActionServer;
  typedef std::shared_ptr<ActionServer>                                                       ActionServerPtr;
  typedef ActionServer::GoalHandle                                                            GoalHandle;

  realtime_tools::RealtimeBuffer< std::vector<hardware_interface::JointCommandModes> > commands_buffer_;
  unsigned int n_joints_;

  std::vector< std::string >                         joint_names_;    ///< Controlled joint names.
  std::vector< hardware_interface::JointModeHandle > joint_modes_;    ///< Handle to controlled joint.
  std::string                                        name_;           ///< Controller name.

  // ROS API
  ros::NodeHandle    controller_nh_;
  ActionServerPtr    action_server_;
  ros::Timer         goal_handle_timer_;
  ros::Duration      action_monitor_period_;

  // Callback, should immediately send result
  void goalCB(GoalHandle gh);

  // General callbacks
  void cancelCB(GoalHandle gh) { /* Do Nothing */ }

};

} // namespace
