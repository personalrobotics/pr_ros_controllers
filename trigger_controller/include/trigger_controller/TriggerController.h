#ifndef TRIGGER_CONTROLLER_TRIGGER_CONTROLLER_H
#define TRIGGER_CONTROLLER_TRIGGER_CONTROLLER_H

#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalStatus.h>
#include <boost/shared_ptr.hpp>
#include <boost/atomic/atomic.hpp>
#include <controller_interface/controller.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pr_hardware_interfaces/TriggerableInterface.h>
#include <pluginlib/class_list_macros.h>
#include <pr_control_msgs/TriggerAction.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <realtime_tools/realtime_box.h>
#include <ros/node_handle.h>


// Ensure atomic integers operations are always lock-free (defined to 2)
#if BOOST_ATOMIC_INT_LOCK_FREE != 2
  #error "Integer atomics not lock-free on this system."
#endif

namespace trigger_controller
{

using pr_hardware_interfaces::TriggerableInterface;

class TriggerController : public controller_interface::Controller<TriggerableInterface>
{
public:

  TriggerController() {}
  ~TriggerController() {}

  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(TriggerableInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  /*\}*/

  /** \name Real-Time Safe Functions
   *\{*/
  void update(const ros::Time& time, const ros::Duration& period);
  /*\}*/


private:

  typedef actionlib::ActionServer<pr_control_msgs::TriggerAction>                  ActionServer;
  typedef boost::shared_ptr<ActionServer>                                          ActionServerPtr;
  typedef ActionServer::GoalHandle                                                 GoalHandle;
  typedef realtime_tools::RealtimeServerGoalHandle<pr_control_msgs::TriggerAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                    RealtimeGoalHandlePtr;


  std::string controller_name_;
  pr_hardware_interfaces::TriggerableHandle triggerable_handle_;

  boost::atomic<pr_hardware_interfaces::TriggerState> trigger_state_;
  realtime_tools::RealtimeBox<RealtimeGoalHandlePtr> rt_goal_;

  ros::Timer service_update_timer_;
  ros::Duration action_monitor_period_;

  ros::NodeHandle controller_nh_;
  ActionServerPtr action_server_;

  void goalCB(GoalHandle gh);

};

} // namespace

#endif // header guard
