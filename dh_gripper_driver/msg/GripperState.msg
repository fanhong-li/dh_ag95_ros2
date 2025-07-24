# DH AG95 Gripper State Message
# Based on DH official gripper state interface

std_msgs/Header header

bool is_initialized     # Whether gripper is initialized
int32 grip_state        # Current grip state (0=moving, 1=gripping, 2=stopped)
float32 position        # Current position (mm)
float32 target_position # Target position (mm)
float32 target_force    # Target force (N)
float32 current_force   # Current measured force (N)
bool object_detected    # Whether object is detected 