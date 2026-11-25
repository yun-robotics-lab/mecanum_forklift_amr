# Fork lift state, published by the lift controller (streaming state -> topic).
# Height is measured from the fork's lowest position (0.0). Units: m, m/s.

std_msgs/Header header

uint8 STATE_UNINITIALIZED = 0
uint8 STATE_HOMING = 1
uint8 STATE_IDLE = 2
uint8 STATE_MOVING = 3
uint8 STATE_FAULT = 4

uint8 FAULT_NONE = 0
uint8 FAULT_TIMEOUT = 1
uint8 FAULT_OVER_LIMIT = 2
uint8 FAULT_NO_FEEDBACK = 3
uint8 FAULT_STALL = 4
uint8 FAULT_SAFETY_STOP = 5

uint8 state              # STATE_*
uint8 fault_code         # FAULT_*; FAULT_NONE unless state == STATE_FAULT
float32 height           # measured fork height [m]
float32 velocity         # measured fork velocity [m/s], positive = up
float32 target_height    # active target [m]; equals height when not moving
bool feedback_available  # false if open-loop (no height feedback)
