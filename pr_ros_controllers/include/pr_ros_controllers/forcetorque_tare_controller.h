#ifndef PR_ROS_CONTROLLERS_FORCETORQUE_TARE_CONTROLLER_H_
#define PR_ROS_CONTROLLERS_FORCETORQUE_TARE_CONTROLLER_H_

#include <actionlib/server/action_server.h>
#include <controller_interface/controller.h>
#include <libbarrett_ros/hardware/ForceTorqueSensorInterface.h>
#include <pluginlib/class_list_macros.h>
#include <pr_ros_controllers/TareAction.h>
#include <ros/node_handle.h>

namespace pr_ros_controllers {

  typedef actionlib::ActionServer<TareAction> TareActionServer;

  class ForceTorqueTareController :
    public controller_interface::Controller<libbarrett_ros::ForceTorqueSensorInterface>
  {
  public:
    ForceTorqueTareController() {}
    ~ForceTorqueTareController() {}

    bool init(libbarrett_ros::ForceTorqueSensorInterface *sensor, ros::NodeHandle &n);
    // void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

  private:
    libbarrett_ros::ForceTorqueSensorHandle sensor_handle_;
    boost::shared_ptr<TareActionServer> tare_as_;
    TareActionServer::GoalHandle active_goal_;
    TareFeedback feedback_;
    TareResult result_;

    void asCallback(TareActionServer::GoalHandle gh);
  };

}

#endif // PR_ROS_CONTROLLERS_FORCETORQUE_TARE_CONTROLLER_H_
