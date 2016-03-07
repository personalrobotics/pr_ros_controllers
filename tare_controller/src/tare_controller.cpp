#include <tare_controller/tare_controller.h>
#include <pr_hardware_interfaces/TareInterface.h>
#include <pluginlib/class_list_macros.h>

namespace tare_controller
{
  typedef TareController<pr_hardware_interfaces::ForceTorqueTareInterface>
          ForceTorqueTareController;
}

PLUGINLIB_EXPORT_CLASS(tare_controller::ForceTorqueTareController, controller_interface::ControllerBase);
