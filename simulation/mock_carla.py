"""
Mock CARLA Classes

Provides lightweight mock implementations of CARLA classes
for running the ADAS system without the actual simulator.
"""

import math
import random


class Vector3D:
    """Mock carla.Vector3D"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar):
        return Vector3D(self.x * scalar, self.y * scalar, self.z * scalar)

    def length(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)


class Location:
    """Mock carla.Location"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def distance(self, other):
        return math.sqrt(
            (self.x - other.x)**2 +
            (self.y - other.y)**2 +
            (self.z - other.z)**2
        )


class Rotation:
    """Mock carla.Rotation"""
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll


class Transform:
    """Mock carla.Transform"""
    def __init__(self, location=None, rotation=None):
        self.location = location or Location()
        self.rotation = rotation or Rotation()


class Waypoint:
    """Mock carla.Waypoint"""
    def __init__(self, transform, lane_id=0, road_id=0):
        self.transform = transform
        self.lane_id = lane_id
        self.road_id = road_id
        self._road_points = []  # Will be set by map

    def next(self, distance):
        """Get waypoints ahead at given distance."""
        # Simple straight road approximation
        yaw_rad = math.radians(self.transform.rotation.yaw)
        new_x = self.transform.location.x + distance * math.cos(yaw_rad)
        new_y = self.transform.location.y + distance * math.sin(yaw_rad)

        new_transform = Transform(
            Location(new_x, new_y, 0),
            Rotation(0, self.transform.rotation.yaw, 0)
        )
        return [Waypoint(new_transform, self.lane_id, self.road_id)]


class LaneType:
    """Mock carla.LaneType"""
    Driving = 1
    Shoulder = 2
    Sidewalk = 3


class VehicleControl:
    """Mock carla.VehicleControl"""
    def __init__(self):
        self.throttle = 0.0
        self.steer = 0.0
        self.brake = 0.0
        self.hand_brake = False
        self.reverse = False
        self.manual_gear_shift = False
        self.gear = 0


class Timestamp:
    """Mock timestamp for world snapshots."""
    def __init__(self, elapsed_seconds=0.0):
        self.elapsed_seconds = elapsed_seconds
        self.frame = 0
        self.delta_seconds = 0.05


class WorldSnapshot:
    """Mock world snapshot."""
    def __init__(self, timestamp):
        self.timestamp = timestamp
