#include <pr_hardware_interfaces/PositionCommandInterface.h>

using hardware_interface::HardwareInterfaceException;

using namespace pr_hardware_interfaces;

PositionCommandHandle::PositionCommandHandle()
  : move_state_(NULL)
  , position_command_(NULL)
{}

PositionCommandHandle::PositionCommandHandle(const std::string& name,
                                             MoveState *move_state,
                                             std::vector<double> *position_command)
  : name_(name)
  , move_state_(move_state)
  , position_command_(position_command)
{
  if (!move_state_) {
    throw HardwareInterfaceException("Cannot create handle '" + name +
                                                         "'. Move state pointer is null.");
  }

  if (!position_command_) {
    throw HardwareInterfaceException("Cannot create handle '" + name +
                                                         "'. Position command pointer is null.");
  }
}

bool PositionCommandHandle::SetCommand(const std::vector<double> &commanded_position)
{
  if (*move_state_ != IDLE) {
    // Cannot command position while moving
    return false;
  }

  if (commanded_position.size() != position_command_->size()) {
    // Commanded position length does not match hardware DOF
    return false;
  }

  for (unsigned int i=0; i < commanded_position.size(); ++i) {
    (*position_command_)[i] = commanded_position[i];
  }

  *move_state_ = MOVE_REQUESTED;
  return true;
}

bool PositionCommandHandle::isDoneMoving()
{
  return *move_state_ == IDLE;
}

