# Complete calibration profile for a user
string username
CalibrationPose[] samples  # Array of calibration samples (T-Pose, forward, up, down)
float64 scale_factor       # User-specific scale factor
builtin_interfaces/Time timestamp
