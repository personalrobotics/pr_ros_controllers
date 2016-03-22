#include <trigger_controller/TriggerController.h>
#include <pluginlib/class_list_macros.h>

namespace trigger_controller
{

namespace
{
std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}
} // namespace

using pr_hardware_interfaces::TRIGGER_IDLE;
using pr_hardware_interfaces::TRIGGER_REQUESTED;
using pr_hardware_interfaces::TRIGGER_PENDING;


bool TriggerController::init(TriggerableInterface* hw,
                          ros::NodeHandle& root_nh,
                          ros::NodeHandle& controller_nh)
{
  controller_nh_ = controller_nh;
  controller_name_ = getLeafNamespace(controller_nh_);

  std::string resource_name;
  if (!controller_nh_.getParam("resource_name", resource_name)) {
    ROS_ERROR_NAMED(controller_name_, "Failed loading resource name from 'resource_name' parameter.");
    return false;
  }

  try {
    triggerable_handle_ = hw->getHandle(resource_name);
  } catch(const std::logic_error& e) {
    ROS_ERROR_STREAM_NAMED(controller_name_, "Unable to initizize controller '" << controller_name_ << "'. " << e.what());
    return false;
  }

  double action_monitor_rate;
  controller_nh_.param("action_monitor_rate", action_monitor_rate, 20.0);
  action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
  ROS_DEBUG_STREAM_NAMED(controller_name_, "Action status changes will be monitored at " <<
                         action_monitor_rate << "Hz.");

  ROS_DEBUG_STREAM_NAMED(controller_name_, "Initialized controller '" << controller_name_ << "' with:" <<
			 "\n- Hardware interface type: '" << this->getHardwareInterfaceType() <<
                         "'" << "\n");

  // ROS API: Action interface
  action_server_.reset(new ActionServer(controller_nh_, "trigger",
					boost::bind(&TriggerController::goalCB, this, _1),
					false));
  action_server_->start();
  ROS_DEBUG_STREAM_NAMED(controller_name_, "ActionServer started");

  return true;
}

void TriggerController::update(const ros::Time& time, const ros::Duration& period)
{
  pr_hardware_interfaces::TriggerState trigger_state = trigger_state_.load();
  RealtimeGoalHandlePtr rt_tmp_gh;
  rt_goal_.get(rt_tmp_gh);

  if (trigger_state == TRIGGER_REQUESTED) {
    if (triggerable_handle_.isTriggerComplete()) {
      triggerable_handle_.trigger();
      trigger_state_.store(TRIGGER_PENDING);
    }
    else {
      rt_tmp_gh->preallocated_result_->success = false;
      rt_tmp_gh->preallocated_result_->message = "Request already triggered.";
      rt_tmp_gh->setAborted(rt_tmp_gh->preallocated_result_);
    }
  }
  else if (trigger_state == TRIGGER_PENDING && triggerable_handle_.isTriggerComplete()) {
    rt_tmp_gh->preallocated_result_->success = true;
    rt_tmp_gh->preallocated_result_->message = "Trigger completed succcessfully.";
    rt_tmp_gh->setSucceeded(rt_tmp_gh->preallocated_result_);
    trigger_state_.store(TRIGGER_IDLE);
  }
}

void TriggerController::goalCB(GoalHandle gh)
{
  ROS_DEBUG_STREAM_NAMED(controller_name_, "Recieved new trigger request.");
  pr_hardware_interfaces::TriggerState trigger_state = trigger_state_.load();

  // Precondition: Running controller
  if (!this->isRunning()) {
    ROS_ERROR_NAMED(controller_name_, "Can't accept new action goals. Controller is not running.");
    pr_control_msgs::TriggerResult result;
    result.success = false;
    result.message = "Controller is not running.";
    gh.setRejected(result);
  }
  else {
    gh.setAccepted();

    RealtimeGoalHandlePtr rt_tmp_gh(new RealtimeGoalHandle(gh));
    // A RealtimeBox is used because shared_ptr is not thread-safe for reset and access
    // operations. We access the same shared_ptr (not a copy) from multiple threads here.
    // In C++11 the atomic functions of std::shared_ptr could be used instead.
    rt_goal_.set(rt_tmp_gh);
    trigger_state_.store(TRIGGER_REQUESTED);

    service_update_timer_ = controller_nh_.createTimer(action_monitor_period_,
                                                       &RealtimeGoalHandle::runNonRealtime,
                                                       rt_tmp_gh);
    ROS_DEBUG_STREAM_NAMED(controller_name_, "Trigger action accepted.");
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS(trigger_controller::TriggerController, controller_interface::ControllerBase);
