#ifndef TARE_CONTROLLER_TARE_CONTROLLER_IMPL_H
#define TARE_CONTROLLER_TARE_CONTROLLER_IMPL_H

namespace tare_controller
{


bool TareController::init(TareInterface* hw,
                          ros::NodeHandle& root_nh,
                          ros::NodeHandle& controller_nh)
{
  controller_nh_ = controller_nh;

  if (!controller_nh_.getParam("tare_name", name_)) {
    ROS_ERROR("Failed loading resource name from 'tare_name' parameter.");
    return false;
  }

  try {
    tare_handle_ = hw->getHandle(name_);
  } catch(const std::logic_error& e) {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to initizize controller '" << name_ << "'. " << e.what());
    return false;
  }

  double action_monitor_rate = 20.0;
  controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
  action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
  ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " <<
                         action_monitor_rate << "Hz.");

  ROS_DEBUG_STREAM_NAMED(name_, "Initialized controller '" << name_ << "' with:" <<
			 "\n- Hardware interface type: '" << this->getHardwareInterfaceType() <<
                         "'" << "\n");

  // pre-alloc result
  result_.reset(new pr_control_msgs::TareResult());
  result_->success = false;
  result_->message = "";

  // ROS API: Action interface
  action_server_.reset(new ActionServer(controller_nh_, "tare",
					boost::bind(&TareController::goalCB, this, _1),
					false));
  action_server_->start();
  ROS_DEBUG_STREAM_NAMED(name_, "ActionServer started");
  return true;
}

void TareController::update(const ros::Time& time, const ros::Duration& period)
{
  if (tare_requested_.load() && tare_handle_.isTareComplete()) {
    ROS_DEBUG_STREAM_NAMED(name_, "Tare complete.");
    result_->success = true;
    result_->message = "Tare completed.";
    rt_active_goal_->setSucceeded(result_);
    tare_requested_.store(false);
  }
}

void TareController::goalCB(GoalHandle gh)
{
  ROS_DEBUG_STREAM_NAMED(name_, "Recieved new tare request.");

  // Precondition: Running controller
  if (!this->isRunning()) {
    ROS_ERROR_NAMED(name_, "Can't accept new action goals. Controller is not running.");
    pr_control_msgs::TareResult result;
    result.success = false;
    result.message = "Controller is not running.";
    gh.setRejected(result);
  }
  // Precondition: Tare not already pending
  else if (tare_requested_.load()) {
    ROS_WARN_NAMED(name_, "Tare already pending, cannot preempt tare requets.");
    pr_control_msgs::TareResult result;
    result.success = false;
    result.message = "Tare already pending, cannot preempt tare requets.";
    gh.setRejected(result);
  }
  else {
    // Try to update goal
    RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));

    gh.setAccepted();
    tare_handle_.tare();

    result_->success = false;
    result_->message = "Action accepted";

    service_update_timer_ = controller_nh_.createTimer(action_monitor_period_,
                                                       &RealtimeGoalHandle::runNonRealtime,
                                                       rt_goal);

    rt_active_goal_ = rt_goal;
    tare_requested_.store(true);
    ROS_DEBUG_STREAM_NAMED(name_, "Tare action accepted.");
  }
}

} // namespace

#endif // header guard
