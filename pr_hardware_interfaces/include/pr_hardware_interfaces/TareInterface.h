#ifndef PR_HARDWARE_INTERFACES_TARE_INTERFACE_H_
#define PR_HARDWARE_INTERFACES_TARE_INTERFACE_H_

#include <string>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <pr_hardware_interfaces/TareState.h>

namespace pr_hardware_interfaces {

class TareHandle
{
public:
  TareHandle();
  TareHandle(const std::string& name, TareState* tare_state);

  std::string getName() const {return name_;}

  void tare();
  bool isTareComplete();

private:
  std::string name_;
  TareState* tare_state_;
};

class TareInterface :
    public hardware_interface::HardwareResourceManager<TareHandle, hardware_interface::ClaimResources>{};

}

#endif // PR_HARDWARE_INTERFACES_TARE_INTERFACE_H_
