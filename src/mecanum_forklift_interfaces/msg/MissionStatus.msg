# Published by the mission node at 2 Hz and on every state change.
std_msgs/Header header
bool active                               # a mission is running
string mission_id
string state                              # IDLE | VALIDATE | PICK | DROP | RETURN | DONE | FAILED | CANCELED
int32 step_index
int32 total_steps
int32 attempt                             # 1 = first try of the current step
string error_code                         # last mission result code ("" while running / on success)
string message
