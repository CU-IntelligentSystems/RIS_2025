# Represents a single detected lane line segment relative to the robot

uint8 COLOR_WHITE=0
uint8 COLOR_YELLOW=1
uint8 COLOR_UNKNOWN=2

uint8 color          # COLOR_WHITE or COLOR_YELLOW
float32 angle        # Angle of the line relative to robot's x-axis (radians)
float32 distance     # Perpendicular distance from robot origin to the line (meters)

