# JODELL EPG Gripper State Message
# Based on registers 0x07D0, 0x07D1, 0x07D2

std_msgs/Header header

# Basic status
bool is_activated          # gACT bit - gripper is activated
bool is_moving             # gripper is currently moving
uint8 object_status        # 0:moving, 1:inner_detected, 2:outer_detected(gripped), 3:arrived

# Position (0-255)
uint8 target_position      # requested position (0=open, 255=close)
uint8 current_position     # actual position from register 0x07D1 high byte

# Speed (0-255)
uint8 target_speed
uint8 current_speed        # actual speed from register 0x07D2 low byte

# Force/Current (0-255) - Key for embodied AI
uint8 target_force         # force limit set in command
uint8 current_force        # actual force/current from register 0x07D2 high byte

# Fault information
uint8 fault_code           # from register 0x07D1 low byte
string fault_message

# Communication status
bool communication_ok
