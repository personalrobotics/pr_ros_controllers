#include <pr_hardware_interfaces/TareInterface.h>

using namespace pr_hardware_interfaces;

TareHandle::TareHandle()
  : tare_state_(NULL)
{}

TareHandle::TareHandle(const std::string& name, TareState* tare_state)
  : name_(name)
  , tare_state_(tare_state)
{
  if (!tare_state_) {
    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                         "'. Tare state pointer is null.");
  }
}

void TareHandle::tare()
{
  if (isTareComplete()) { // ignore double tares
    *tare_state_ = TARE_REQUESTED;
  }
}

bool TareHandle::isTareComplete()
{
  assert(tare_state_);
  return *tare_state_ == TARE_IDLE;
}

