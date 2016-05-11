#ifndef PR_HARDWARE_INTERFACES_MOVESTATE_H_
#define PR_HARDWARE_INTERFACES_MOVESTATE_H_

namespace pr_hardware_interfaces {

/// \brief Internal state of actuatable harware
///
/// IDLE: active and waiting for request
/// MOVE_REQUESTED: move request has been registered, but
///                 hardware motion not begun
/// MOVING: hardware movement commands sent, currently moving
/// IN_ERROR: hardware in an error or inconsistent state
enum MoveState { IDLE, MOVE_REQUESTED, MOVING, IN_ERROR };

}

#endif
