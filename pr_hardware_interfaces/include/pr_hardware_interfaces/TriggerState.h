#ifndef PR_HARDWARE_INTERFACE_TRIGGERSTATE_H_
#define PR_HARDWARE_INTERFACE_TRIGGERSTATE_H_

namespace pr_hardware_interfaces {

/// \brief State of the Trigger request
///
/// TRIGGER_IDLE: no request active
/// TRIGGER_REQUESTED: trigger request received but not currently processing
/// TRIGGER_PENDING: trigger request underway
enum TriggerState { TRIGGER_IDLE, TRIGGER_REQUESTED, TRIGGER_PENDING };

}

#endif
