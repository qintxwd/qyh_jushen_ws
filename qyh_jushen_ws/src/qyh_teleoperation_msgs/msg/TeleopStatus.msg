std_msgs/Header header

uint8 IDLE = 0
uint8 TRACKING = 1
uint8 PAUSED = 2
uint8 ERROR = 3

uint8 status
string message
float64 tracking_quality  # 0.0-1.0
