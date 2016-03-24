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
  // TODO use "resource_name" in other controllers as well
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

  double action_monitor_rate;
  controller_nh_.param("action_monitor_rate", action_monitor_rate, 20.0);
  action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
  ROS_DEBUG_STREAM_NAMED(controller_name_, "Action status changes will be monitored at " <<
                         action_monitor_rate << "Hz.");

  ROS_DEBUG_STREAM_NAMED(controller_name_, "Initialized controller '" <<
                         controller_name_ << "' with:" << "\n- Hardware interface type: '" <<
                         this->getHardwareInterfaceType() << "'" << "\n");

  action_server_.reset(new ActionServer(controller_nh_, "move_hand",
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
  rt_goal_.get(rt_tmp_gh);

  if (move_state == MOVE_REQUESTED) {
    if (cmd_handle_.isDoneMoving()) {
      auto goal = rt_tmp_gh->gh_.getGoal();
      if (goal->command.position.size() != cmd_handle_.getNumDof()) {
        // check command validity
        // TODO command range?
        rt_tmp_gh->preallocated_result_->success = false;
        rt_tmp_gh->preallocated_result_->message = "Commanded position does not match degrees of freedom.";
        rt_tmp_gh->setAborted(rt_tmp_gh->preallocated_result_);
        move_state_.store(IDLE);
      }
      else {
        // Execute command
        // TODO correct command position order by names
        cmd_handle_.setCommand(goal->command.position);
        move_state_.store(MOVING);
      }
    }
    else {
      // UNEXPECTED STATE: another controller has triggered a command
      // TODO remove once task/hardware interface properly checks for IN_ERROR state
      rt_tmp_gh->preallocated_result_->success = false;
      rt_tmp_gh->preallocated_result_->message = "Command failed. Hardware moving unexpectedly.";
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
  pr_hardware_interfaces::MoveState move_state = move_state_.load();

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
      result.message = "Command pending and not interruptable.";
      gh.setRejected(result);
      ROS_DEBUG_STREAM_NAMED(controller_name_, "Position command rejected.");
    }
    else {
      gh.setAccepted();
      RealtimeGoalHandlePtr rt_tmp_gh = boost::make_shared<RealtimeGoalHandle>(gh);
      rt_goal_.set(rt_tmp_gh);
      move_state_.store(MOVE_REQUESTED);

      service_update_timer_ = controller_nh_.createTimer(action_monitor_period_,
                                                         &RealtimeGoalHandle::runNonRealtime,
                                                         rt_tmp_gh);
      ROS_DEBUG_STREAM_NAMED(controller_name_, "Position command accepted.");
    }
  }
}

PLUGINLIB_EXPORT_CLASS(position_command_controller::PositionCommandController, controller_interface::ControllerBase);
