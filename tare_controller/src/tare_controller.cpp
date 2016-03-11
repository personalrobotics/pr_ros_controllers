#include <tare_controller/tare_controller.h>
#include <pluginlib/class_list_macros.h>

namespace tare_controller
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

using pr_hardware_interfaces::TARE_IDLE;
using pr_hardware_interfaces::TARE_REQUESTED;
using pr_hardware_interfaces::TARE_PENDING;


bool TareController::init(TareInterface* hw,
                          ros::NodeHandle& root_nh,
                          ros::NodeHandle& controller_nh)
{
  controller_nh_ = controller_nh;
  controller_name_ = getLeafNamespace(controller_nh_);

  std::string tare_handle_name;
  if (!controller_nh_.getParam("tare_handle_name", tare_handle_name)) {
    ROS_ERROR_NAMED(controller_name_, "Failed loading resource name from 'tare_handle_name' parameter.");
    return false;
  }

  try {
    tare_handle_ = hw->getHandle(tare_handle_name);
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
  action_server_.reset(new ActionServer(controller_nh_, "tare",
					boost::bind(&TareController::goalCB, this, _1),
					false));
  action_server_->start();
  ROS_DEBUG_STREAM_NAMED(controller_name_, "ActionServer started");

  return true;
}

void TareController::update(const ros::Time& time, const ros::Duration& period)
{
  pr_hardware_interfaces::TareState tare_state = tare_state_.load();
  RealtimeGoalHandlePtr rt_tmp_gh;
  rt_goal_.get(rt_tmp_gh);

  if (tare_state == TARE_REQUESTED) {
    if (tare_handle_.isTareComplete()) {
      tare_handle_.tare();
      tare_state_.store(TARE_PENDING);
    }
    else {
      rt_tmp_gh->preallocated_result_->success = false;
      rt_tmp_gh->preallocated_result_->message = "Tare is already in progress.";
      rt_tmp_gh->setAborted(rt_tmp_gh->preallocated_result_);
    }
  }
  else if (tare_state == TARE_PENDING && tare_handle_.isTareComplete()) {
    rt_tmp_gh->preallocated_result_->success = true;
    rt_tmp_gh->preallocated_result_->message = "Tare completed succcessfully.";
    rt_tmp_gh->setSucceeded(rt_tmp_gh->preallocated_result_);
    tare_state_.store(TARE_IDLE);
  }
}

void TareController::goalCB(GoalHandle gh)
{
  ROS_DEBUG_STREAM_NAMED(controller_name_, "Recieved new tare request.");
  pr_hardware_interfaces::TareState tare_state = tare_state_.load();

  // Precondition: Running controller
  if (!this->isRunning()) {
    ROS_ERROR_NAMED(controller_name_, "Can't accept new action goals. Controller is not running.");
    pr_control_msgs::TareResult result;
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
    tare_state_.store(TARE_REQUESTED);

    service_update_timer_ = controller_nh_.createTimer(action_monitor_period_,
                                                       &RealtimeGoalHandle::runNonRealtime,
                                                       rt_tmp_gh);
    ROS_DEBUG_STREAM_NAMED(controller_name_, "Tare action accepted.");
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS(tare_controller::TareController, controller_interface::ControllerBase);
