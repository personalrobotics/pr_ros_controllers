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

  if (!nh.getParam("publish_rate", publish_rate_)){
    ROS_ERROR("Failed to load 'publish_rate' parameter.");
    return false;
  }

  ROS_INFO("Acquiring sensor hardware handle.");
  sensor_ = sensor->getHandle(ft_name); // TODO try/catch?

  ROS_INFO("Starting action server.");
  tare_as_.reset(new TareActionServer(nh, "tare_controller",
                                      boost::bind(&ForceTorqueTareController::asCallback, this, _1),
                                      false));
  tare_as_->start();

  ROS_INFO("Initializing realtime publisher.");
  ft_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh, ft_name, 4));

  ROS_INFO("Tare controller '%s' started.", ft_name.c_str());
  return true;
}

void ForceTorqueTareController::starting(const ros::Time& time)
{
  last_publish_time_ = time;
}

void ForceTorqueTareController::update(const ros::Time& time, const ros::Duration& period)
{
  // Update tare status
  if (feedback_.requested) {
    feedback_.finished = sensor_.isTareComplete();
    if (feedback_.finished) {
      result_.success = true; // no way to confirm success from hardware
      feedback_.requested = false;
      active_goal_.setSucceeded(result_);
    }
    else {
      active_goal_.publishFeedback(feedback_); // TODO reduce publish rate?
    }
  }

  // publish wrench
  if (publish_rate_ > 0.0
      && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time) {
    if (ft_pub_->trylock()) {
      last_publish_time_ += ros::Duration(1.0/publish_rate_);

      ft_pub_->msg_.header.stamp = time;
      ft_pub_->msg_.header.frame_id = sensor_.getFrameId(); // TODO

      ft_pub_->msg_.wrench.force.x = sensor_.getForce()[0];
      ft_pub_->msg_.wrench.force.y = sensor_.getForce()[1];
      ft_pub_->msg_.wrench.force.z = sensor_.getForce()[2];
      ft_pub_->msg_.wrench.torque.x = sensor_.getTorque()[0];
      ft_pub_->msg_.wrench.torque.y = sensor_.getTorque()[1];
      ft_pub_->msg_.wrench.torque.z = sensor_.getTorque()[2];

      ft_pub_->unlockAndPublish();
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
  sensor_.tare();
}

PLUGINLIB_EXPORT_CLASS(pr_ros_controllers::ForceTorqueTareController, controller_interface::ControllerBase)
