#ifndef POSITION_COMMAND_CONTROLLER_POSITION_COMMAND_CONTROLLER_H_
#define POSITION_COMMAND_CONTROLLER_POSITION_COMMAND_CONTROLLER_H_

#include <actionlib/server/action_server.h>
#include <boost/atomic/atomic.hpp>
#include <boost/shared_ptr.hpp>
#include <controller_interface/controller.h>
#include <pr_control_msgs/SetPositionAction.h>
#include <pr_hardware_interfaces/MoveState.h>
#include <pr_hardware_interfaces/PositionCommandInterface.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <realtime_tools/realtime_box.h>
#include <ros/node_handle.h>

// Ensure atomic integers operations are always lock-free (defined to 2)
#if BOOST_ATOMIC_INT_LOCK_FREE != 2
  #error "Integer atomics not lock-free on this system."
#endif


namespace position_command_controller
{

using pr_control_msgs::SetPositionAction;
using pr_hardware_interfaces::PositionCommandInterface;

class PositionCommandController
: public controller_interface::Controller<PositionCommandInterface>
{
public:

  PositionCommandController() {}
  ~PositionCommandController() {}

  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(PositionCommandInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
  /*\}*/

  /** \name Real-Time Safe Functions
   *\{*/
  void update(const ros::Time &time, const ros::Duration &period);
  /*\}*/

private:

  typedef actionlib::ActionServer<SetPositionAction>                  ActionServer;
  typedef boost::shared_ptr<ActionServer>                             ActionServerPtr;
  typedef ActionServer::GoalHandle                                    GoalHandle;
  typedef realtime_tools::RealtimeServerGoalHandle<SetPositionAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                       RealtimeGoalHandlePtr;

  std::string controller_name_;
  pr_hardware_interfaces::PositionCommandHandle cmd_handle_;

  boost::atomic<pr_hardware_interfaces::MoveState> move_state_;
  realtime_tools::RealtimeBox<RealtimeGoalHandlePtr> rt_goal_;

  ros::Timer service_update_timer_;
  ros::Duration action_monitor_period_;

  ros::NodeHandle controller_nh_;
  ActionServerPtr action_server_;

  void goalCB(GoalHandle gh);
  
};

} // namespace

#endif
