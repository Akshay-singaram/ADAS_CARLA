"""
ADAS Configuration Parameters
"""

# CARLA Connection
CARLA_HOST = 'localhost'
CARLA_PORT = 2000
CARLA_TIMEOUT = 10.0

# Vehicle Settings
TARGET_SPEED = 30.0  # km/h
MAX_SPEED = 50.0  # km/h
MIN_SPEED = 0.0  # km/h

# Lane Following Parameters
LANE_FOLLOWING_K_P = 1.0  # Proportional gain for steering
LANE_FOLLOWING_K_D = 0.1  # Derivative gain for steering
LOOKAHEAD_DISTANCE = 5.0  # meters ahead to look for waypoints

# Adaptive Cruise Control Parameters
SAFE_DISTANCE = 10.0  # minimum safe distance in meters
TIME_GAP = 2.0  # time gap in seconds (distance = speed * time_gap)
ACC_K_P = 0.5  # Proportional gain for throttle/brake
ACC_K_D = 0.1  # Derivative gain for throttle/brake
EMERGENCY_BRAKE_DISTANCE = 5.0  # meters - triggers emergency brake
EMERGENCY_BRAKE_TTC = 1.5  # Time-to-collision threshold in seconds

# Cut-in Detection Parameters
CUTIN_LANE_THRESHOLD = 2.0  # meters from lane center to detect cut-in
CUTIN_DISTANCE_THRESHOLD = 30.0  # meters ahead to monitor for cut-ins
CUTIN_LATERAL_VELOCITY_THRESHOLD = 0.5  # m/s lateral velocity indicating lane change

# Sensor Settings
RADAR_RANGE = 100.0  # meters
RADAR_FOV = 30.0  # degrees (horizontal field of view)

# Control Loop
CONTROL_FREQUENCY = 20  # Hz
