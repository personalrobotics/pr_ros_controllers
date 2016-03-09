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
  TareHandle()
    : tare_state_(NULL)
  {}

  TareHandle(const std::string& name, TareState* tare_state)
    : name_(name)
    , tare_state_(tare_state)
  {
    if (!tare_state_) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Tare command pointer is null.");
    }
  }

  std::string getName() const {return name_;}

  void tare()
  {
    if (isTareComplete()) { // ignore double tares
      *tare_state_ = TARE_REQUESTED;
    }
  }

  bool isTareComplete()
  {
    assert(tare_state_);
    return *tare_state_ == TARE_IDLE;
  }

private:
  std::string name_;
  TareState* tare_state_;
};

class TareInterface :
    public hardware_interface::HardwareResourceManager<TareHandle, hardware_interface::ClaimResources>{};

}

#endif // PR_HARDWARE_INTERFACES_TARE_INTERFACE_H_
