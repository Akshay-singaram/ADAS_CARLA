"""
Helper utility functions for ADAS
"""

import math


def get_speed(vehicle):
    """
    Get vehicle speed in km/h.

    Args:
        vehicle: CARLA vehicle actor or SimulatedVehicle

    Returns:
        float: Speed in km/h
    """
    # Check if it's a simulated vehicle with get_speed_kmh method
    if hasattr(vehicle, 'get_speed_kmh'):
        return vehicle.get_speed_kmh()

    # CARLA vehicle - get velocity vector
    velocity = vehicle.get_velocity()
    speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    return speed_ms * 3.6  # Convert m/s to km/h


def get_vehicle_transform(vehicle):
    """
    Get vehicle transform (location and rotation).

    Args:
        vehicle: CARLA vehicle actor

    Returns:
        carla.Transform: Vehicle transform
    """
    return vehicle.get_transform()


def calculate_distance(location1, location2):
    """
    Calculate Euclidean distance between two locations.

    Args:
        location1: First CARLA location
        location2: Second CARLA location

    Returns:
        float: Distance in meters
    """
    return math.sqrt(
        (location1.x - location2.x)**2 +
        (location1.y - location2.y)**2 +
        (location1.z - location2.z)**2
    )


def calculate_distance_2d(location1, location2):
    """
    Calculate 2D Euclidean distance between two locations (ignoring Z).

    Args:
        location1: First CARLA location
        location2: Second CARLA location

    Returns:
        float: Distance in meters
    """
    return math.sqrt(
        (location1.x - location2.x)**2 +
        (location1.y - location2.y)**2
    )


def normalize_angle(angle):
    """
    Normalize angle to [-180, 180] degrees.

    Args:
        angle: Angle in degrees

    Returns:
        float: Normalized angle in degrees
    """
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def get_relative_position(ego_transform, target_location):
    """
    Get target position relative to ego vehicle.

    Args:
        ego_transform: Ego vehicle transform
        target_location: Target CARLA location

    Returns:
        tuple: (longitudinal, lateral) distances in meters
               longitudinal: positive = ahead, negative = behind
               lateral: positive = right, negative = left
    """
    ego_loc = ego_transform.location
    ego_yaw = math.radians(ego_transform.rotation.yaw)

    # Vector from ego to target
    dx = target_location.x - ego_loc.x
    dy = target_location.y - ego_loc.y

    # Rotate to ego frame
    longitudinal = dx * math.cos(ego_yaw) + dy * math.sin(ego_yaw)
    lateral = -dx * math.sin(ego_yaw) + dy * math.cos(ego_yaw)

    return longitudinal, lateral


def get_relative_velocity(ego_velocity, target_velocity, ego_yaw):
    """
    Get target velocity relative to ego vehicle.

    Args:
        ego_velocity: Ego vehicle velocity (carla.Vector3D)
        target_velocity: Target vehicle velocity (carla.Vector3D)
        ego_yaw: Ego vehicle yaw in degrees

    Returns:
        tuple: (longitudinal, lateral) relative velocities in m/s
    """
    ego_yaw_rad = math.radians(ego_yaw)

    # Relative velocity in world frame
    rel_vx = target_velocity.x - ego_velocity.x
    rel_vy = target_velocity.y - ego_velocity.y

    # Rotate to ego frame
    longitudinal = rel_vx * math.cos(ego_yaw_rad) + rel_vy * math.sin(ego_yaw_rad)
    lateral = -rel_vx * math.sin(ego_yaw_rad) + rel_vy * math.cos(ego_yaw_rad)

    return longitudinal, lateral


def clamp(value, min_value, max_value):
    """
    Clamp a value between min and max.

    Args:
        value: Value to clamp
        min_value: Minimum value
        max_value: Maximum value

    Returns:
        float: Clamped value
    """
    return max(min_value, min(max_value, value))
