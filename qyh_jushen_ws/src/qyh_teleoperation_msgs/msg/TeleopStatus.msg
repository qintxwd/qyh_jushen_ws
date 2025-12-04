std_msgs/Header header

uint8 IDLE = 0
uint8 TRACKING = 1
uint8 PAUSED = 2
uint8 ERROR = 3
uint8 CLUTCH_ENGAGING = 4
uint8 CLUTCH_RELEASING = 5

uint8 status
string message
float64 tracking_quality  # 0.0-1.0

# Clutch状态
bool left_clutch_engaged
bool right_clutch_engaged
float64 left_grip_value
float64 right_grip_value
