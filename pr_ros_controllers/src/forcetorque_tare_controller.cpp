#include <pr_ros_controllers/forcetorque_tare_controller.h>

using namespace pr_ros_controllers;

bool ForceTorqueTareController::init(
  libbarrett_ros::ForceTorqueSensorInterface *sensor, ros::NodeHandle &nh)
{
  ROS_INFO("Loading ft_name from config.");
  std::string ft_name;
  if (!nh.getParam("ft_name", ft_name)) {
    ROS_ERROR("Failed to load 'ft_name' parameter.");
    return false;
  }

  sensor_handle_ = sensor->getHandle(ft_name); // TODO try/catch?

  tare_as_.reset(new TareActionServer(nh, "tare_controller",
                                      boost::bind(&ForceTorqueTareController::asCallback, this, _1),
                                      false));
  tare_as_->start();
  return true;
}

void ForceTorqueTareController::update(const ros::Time& time, const ros::Duration& period)
{
  if (feedback_.requested) {
    feedback_.finished = sensor_handle_.isTareComplete();
    if (feedback_.finished) {
      result_.success = true; // no way to confirm success from hardware
      feedback_.requested = false;
      tare_as_->setSucceeded(result_);
    }
    else {
      tare_as_->publishFeedback(feedback_); // TODO reduce publish rate?
    }
  }
}

void ForceTorqueTareController::asCallback(const TareGoalConstPtr &goal)
{
  sensor_handle_.tare();
  feedback_.requested = true;
  // TODO need to "accept" goal?
}
