#ifndef TARE_CONTROLLER_TARE_CONTROLLER_IMPL_H
#define TARE_CONTROLLER_TARE_CONTROLLER_IMPL_H

namespace tare_controller
{
namespace internal
{

std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

} // namespace


template <class HardwareInterface>
bool TareController<HardwareInterface>::
init(HardwareInterface* hw,
     ros::NodeHandle&   root_nh,
     ros::NodeHandle&   controller_nh)
{
  using namespace internal;

  controller_nh_ = controller_nh;
  name_ = getLeafNamespace(controller_nh_);
  hw_ = hw; //->getHandle(name_); // TODO try/catch?

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

template <class HardwareInterface>
void TareController<HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  if (tare_requested_.load() && hw_->getHandle(name_).isTareComplete()) { // TODO not right (valid)
    result_->success = true;
    result_->message = "Tare completed.";
    rt_active_goal_->setSucceeded(result_);
    tare_requested_.store(0);
  }
}

template <class HardwareInterface>
void TareController<HardwareInterface>::
goalCB(GoalHandle gh)
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
    hw_->getHandle(name_).tare();

    result_->success = false;
    result_->message = "Action accepted";

    rt_active_goal_ = rt_goal;
    tare_requested_.store(1);
  }
}

} // namespace

#endif // header guard
