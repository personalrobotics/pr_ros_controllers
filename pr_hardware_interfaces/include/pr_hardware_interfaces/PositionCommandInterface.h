#ifndef PR_HARDWARE_INTERFACES_POSITION_COMMAND_INTERFACE_H_
#define PR_HARDWARE_INTERFACES_POSITION_COMMAND_INTERFACE_H_

#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <pr_hardware_interfaces/MoveState.h>

namespace pr_hardware_interfaces {

class PositionCommandHandle
{
public:
  PositionCommandHandle();
  PositionCommandHandle(const std::string& name,
                        MoveState *move_state,
                        std::vector<double> *position_command);

  std::string getName() const;
  size_t getNumDof() const;
  MoveState getState() const;
  bool setCommand(const std::vector<double> &commanded_position);
  bool isDoneMoving();

private:
  std::string name_;
  MoveState *move_state_;
  std::vector<double> *position_command_;
};

class PositionCommandInterface :
    public hardware_interface::HardwareResourceManager<PositionCommandHandle, hardware_interface::ClaimResources>{};

} // namespace

#endif
