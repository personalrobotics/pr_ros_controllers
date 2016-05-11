#ifndef PR_HARDWARE_INTERFACES_TRIGGERABLE_INTERFACE_H_
#define PR_HARDWARE_INTERFACES_TRIGGERABLE_INTERFACE_H_

#include <string>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <pr_hardware_interfaces/TriggerState.h>

namespace pr_hardware_interfaces {

class TriggerableHandle
{
public:
  TriggerableHandle();
  TriggerableHandle(const std::string& name, TriggerState* trigger_state);

  std::string getName() const;
  void trigger();
  bool isTriggerComplete();

private:
  std::string name_;
  TriggerState* trigger_state_;
};

class TriggerableInterface :
    public hardware_interface::HardwareResourceManager<TriggerableHandle, hardware_interface::ClaimResources>{};

}

#endif // PR_HARDWARE_INTERFACES_TRIGGERABLE_INTERFACE_H_
