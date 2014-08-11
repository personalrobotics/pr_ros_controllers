/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Carnegie Mellon University
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

/* requires:
#include <ros/node_handle.h>
#include <hardware_interface/joint_desired_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
*/

namespace pr_ros_controllers
{
   
/**
 * \brief PR Joint Velocity Controller (linear or angular)
 *
 * This class passes the commanded velocity down to the joint and does PID
 *
 * \section ROS interface
 *
 * \param type Must be "PrJointVelocityController".
 * \param joint Name of the joint to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64) : The joint velocity to apply
 */

class PrJointVelocityController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  PrJointVelocityController() : desired_(0), last_e_(0) {}
  ~PrJointVelocityController() {sub_desired_.shutdown();}

  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n)
  {
    std::string joint_name;
    if (!n.getParam("joint", joint_name))
    {
      ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
      return false;
    }

    joint_ = hw->getHandle(joint_name);
    sub_desired_ = n.subscribe<std_msgs::Float64>("command", 1, &PrJointVelocityController::commandCB, this);

    if (!n.getParam("p", p_))
    {
      ROS_ERROR("No P gain given (namespace: %s)", n.getNamespace().c_str());
      return false;
    }

    if (!n.getParam("i", i_))
    {
      ROS_WARN("No I gain given (namespace: %s); defaulting to 0.", n.getNamespace().c_str());
      i_ = 0;
    }

    if (!n.getParam("d", d_))
    {
      ROS_WARN("No D gain given (namespace: %s); defaulting to 0.", n.getNamespace().c_str());
      d_ = 0;
    }
    return true;
  }

  void starting(const ros::Time& time) {desired_ = 0;}
  void update(const ros::Time& time, const ros::Duration& period)
  {
    // TODO: Currently just does P and D, not I.
    double current_vel = joint_.getVelocity();
    double e = desired_ - current_vel;
    joint_.setCommand(p_*e + d_*(e - last_e_));
    last_e_ = e;
  }

  hardware_interface::JointHandle joint_;
  double desired_;

private:
  double last_e_;

  // PID gains
  double p_;
  double i_;
  double d_;

  ros::Subscriber sub_desired_;
  void commandCB(const std_msgs::Float64ConstPtr& msg) {desired_ = msg->data;}
};

}; /* namespace pr_ros_controllers */
