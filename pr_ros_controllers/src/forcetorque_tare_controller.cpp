#include <pr_ros_controllers/forcetorque_tare_controller.h>

using namespace pr_ros_controllers;

bool ForceTorqueTareController::init(
  libbarrett_ros::ForceTorqueSensorInterface *sensor, ros::NodeHandle &nh)
{
  ROS_INFO("Tare controller starting under namespace: '%s'", nh.getNamespace().c_str());
  ROS_INFO("Loading ft_name from config.");
  std::string ft_name;
  if (!nh.getParam("ft_name", ft_name)) {
    ROS_ERROR("Failed to load 'ft_name' parameter.");
    return false;
  }

  ROS_INFO("Acquiring sensor hardware handle.");
  sensor_handle_ = sensor->getHandle(ft_name); // TODO try/catch?

  ROS_INFO("Starting action server.");
  tare_as_.reset(new TareActionServer(nh, "tare_controller",
                                      boost::bind(&ForceTorqueTareController::asCallback, this, _1),
                                      false));
  tare_as_->start();
  ROS_INFO("Tare controller '%s' started.", ft_name.c_str());
  return true;
}

void ForceTorqueTareController::update(const ros::Time& time, const ros::Duration& period)
{
  if (feedback_.requested) {
    feedback_.finished = sensor_handle_.isTareComplete();
    if (feedback_.finished) {
      result_.success = true; // no way to confirm success from hardware
      feedback_.requested = false;
      active_goal_.setSucceeded(result_);
    }
    else {
      active_goal_.publishFeedback(feedback_); // TODO reduce publish rate?
    }
  }
}

void ForceTorqueTareController::asCallback(TareActionServer::GoalHandle gh)
{
  ROS_INFO("Tare action called.");
  if (feedback_.requested) {
    // TODO complain
    ROS_WARN("Tare called twice.");
  }
  active_goal_ = gh;
  active_goal_.setAccepted();
  feedback_.requested = true;
  sensor_handle_.tare();
}

PLUGINLIB_EXPORT_CLASS(pr_ros_controllers::ForceTorqueTareController, controller_interface::ControllerBase)
