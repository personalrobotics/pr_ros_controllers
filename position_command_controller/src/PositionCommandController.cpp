#include <position_command_controller/PositionCommandController.h>
#include <pluginlib/class_list_macros.h>

using namespace position_command_controller;

namespace
{
std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}
} // namespace

using pr_hardware_interfaces::MoveState;
using pr_hardware_interfaces::IDLE;
using pr_hardware_interfaces::MOVE_REQUESTED;
using pr_hardware_interfaces::MOVING;
using pr_hardware_interfaces::IN_ERROR;

bool PositionCommandController::init(PositionCommandInterface *hw,
                                     ros::NodeHandle &root_nh,
                                     ros::NodeHandle &controller_nh)
{
  controller_nh_ = controller_nh;
  controller_name_ = getLeafNamespace(controller_nh_);

  std::string cmd_handle_name;
  if (!controller_nh_.getParam("resource_name", cmd_handle_name)) {
    ROS_ERROR_NAMED(controller_name_, "Failed loading resource name from 'resource_name' parameter.");
    return false;
  }

  try {
    cmd_handle_ = hw->getHandle(cmd_handle_name);
  } catch(const std::logic_error& e) {
    ROS_ERROR_STREAM_NAMED(controller_name_, "Unable to initizize controller '" << controller_name_ << "'. " << e.what());
    return false;
  }
  // get DOF size once to use in non-realtime thread
  hw_dof_ = cmd_handle_.getNumDof();

  move_state_.store(IDLE);

  double action_monitor_rate;
  controller_nh_.param("action_monitor_rate", action_monitor_rate, 20.0);
  action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
  ROS_DEBUG_STREAM_NAMED(controller_name_, "Action status changes will be monitored at " <<
                         action_monitor_rate << " Hz.");

  ROS_DEBUG_STREAM_NAMED(controller_name_, "Initialized controller '" <<
                         controller_name_ << "' with:" << "\n- Hardware interface type: '" <<
                         this->getHardwareInterfaceType() << "'" << "\n");

  action_server_.reset(new ActionServer(controller_nh_, "set_position",
					boost::bind(&PositionCommandController::goalCB, this, _1),
					false));
  action_server_->start();
  ROS_DEBUG_STREAM_NAMED(controller_name_, "ActionServer started");

  return true;
}


void PositionCommandController::update(const ros::Time &time, const ros::Duration &period)
{
  pr_hardware_interfaces::MoveState move_state = move_state_.load();
  RealtimeGoalHandlePtr rt_tmp_gh;
  rt_gh_.get(rt_tmp_gh);
  GoalConstPtr rt_tmp_goal;
  rt_goal_.get(rt_tmp_goal);

  if (move_state == MOVE_REQUESTED) {
    if (cmd_handle_.isDoneMoving()) {
      // Execute command
      // TODO correct command position order by names
      if (!cmd_handle_.setCommand(rt_tmp_goal->command.position)) {
        rt_tmp_gh->preallocated_result_->success = false;
        rt_tmp_gh->preallocated_result_->message = "Command rejected by hardware interface.";
        rt_tmp_gh->setAborted(rt_tmp_gh->preallocated_result_);
        move_state_.store(IDLE);
      }
      else {
        move_state_.store(MOVING);
      }
    }
    else {
      // UNEXPECTED STATE: another controller has triggered a command
      // TODO remove once task/hardware interface properly checks for IN_ERROR state
      rt_tmp_gh->preallocated_result_->success = false;
      rt_tmp_gh->preallocated_result_->message =
        "Command failed because another command is in progress. Is another controller using this hardware?";
      rt_tmp_gh->setAborted(rt_tmp_gh->preallocated_result_);
      move_state_.store(IDLE);
    }
  }
  else if (move_state == MOVING && cmd_handle_.isDoneMoving()) {
    rt_tmp_gh->preallocated_result_->success = true;
    rt_tmp_gh->preallocated_result_->message = "Move completed successfully.";
    rt_tmp_gh->setSucceeded(rt_tmp_gh->preallocated_result_);
    move_state_.store(IDLE);
  }
  else if (move_state == IN_ERROR) {
    rt_tmp_gh->preallocated_result_->success = false;
    rt_tmp_gh->preallocated_result_->message = "Command failed. Hardware in unexpected state.";
    rt_tmp_gh->setAborted(rt_tmp_gh->preallocated_result_);
    move_state_.store(IDLE);  // hope for the best next cycle
  }
}


void PositionCommandController::goalCB(GoalHandle gh)
{
  ROS_DEBUG_NAMED(controller_name_, "Recieved new position command.");
  if (!this->isRunning()) {
    ROS_ERROR_NAMED(controller_name_, "Can't accept new action goals. Controller is not running.");
    pr_control_msgs::SetPositionResult result;
    result.success = false;
    result.message = "Controller is not running.";
    gh.setRejected(result);
  }
  else {
    MoveState move_state = move_state_.load();
    if (move_state != IDLE) {
      pr_control_msgs::SetPositionResult result;
      result.success = false;
      result.message = "Command rejected because another command is in progress.";
      gh.setRejected(result);
      ROS_DEBUG_STREAM_NAMED(controller_name_, "Position command rejected.");
    }
    else if (gh.getGoal()->command.position.size() != hw_dof_) {
      pr_control_msgs::SetPositionResult result;
      result.success = false;
      std::stringstream msg;
      msg << "Commanded position length: " << gh.getGoal()->command.position.size()
          << ", does not match degrees of freedom: " << hw_dof_;
      result.message = msg.str();
      gh.setRejected(result);
    }
    else {
      gh.setAccepted();
      RealtimeGoalHandlePtr rt_tmp_gh(new RealtimeGoalHandle(gh));
      rt_gh_.set(rt_tmp_gh);
      rt_goal_.set(gh.getGoal());
      move_state_.store(MOVE_REQUESTED);

      service_update_timer_ = controller_nh_.createTimer(action_monitor_period_,
                                                         &RealtimeGoalHandle::runNonRealtime,
                                                         rt_tmp_gh);
      ROS_DEBUG_STREAM_NAMED(controller_name_, "Position command accepted.");
    }
  }
}

PLUGINLIB_EXPORT_CLASS(position_command_controller::PositionCommandController, controller_interface::ControllerBase);
