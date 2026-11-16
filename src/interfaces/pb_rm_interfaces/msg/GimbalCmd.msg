std_msgs/Header header

# constants for control type
uint8 ABSOLUTE_ANGLE = 1    # position control, set position by absolute angle
uint8 VELOCITY = 2          # velocity control, set velocity

# control type
uint8 yaw_type 1
uint8 pitch_type 1

# control dada
Gimbal position
Gimbal velocity

# command source for arbitration in gimbal_manager
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_BEHAVIOR=1
uint8 SOURCE_TELEOP=2
uint8 SOURCE_VISION=3

uint8 source 0

# optional feed-forward acceleration from the planner (rad/s^2)
float32 yaw_acceleration 0.0
float32 pitch_acceleration 0.0
