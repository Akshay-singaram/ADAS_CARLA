"""
Lane Following Module

Uses CARLA waypoints to follow the current lane with PD steering control.
"""

import math
from utils.helpers import normalize_angle, calculate_distance_2d, clamp
import config


class LaneFollower:
    """
    Lane following controller using waypoint-based steering.
    """

    def __init__(self, vehicle, world_map):
        """
        Initialize the lane follower.

        Args:
            vehicle: CARLA ego vehicle actor
            world_map: CARLA world map
        """
        self.vehicle = vehicle
        self.map = world_map
        self.previous_error = 0.0

        # PD controller gains
        self.k_p = config.LANE_FOLLOWING_K_P
        self.k_d = config.LANE_FOLLOWING_K_D
        self.lookahead_distance = config.LOOKAHEAD_DISTANCE

    def get_steering(self, dt):
        """
        Calculate steering command to follow the lane.

        Args:
            dt: Time delta since last update

        Returns:
            float: Steering value in range [-1, 1]
        """
        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_yaw = vehicle_transform.rotation.yaw

        # Get current waypoint
        current_waypoint = self.map.get_waypoint(
            vehicle_location,
            project_to_road=True,
            lane_type=self._get_lane_type()
        )

        if current_waypoint is None:
            return 0.0

        # Get target waypoint ahead
        target_waypoint = self._get_lookahead_waypoint(current_waypoint)

        if target_waypoint is None:
            return 0.0

        # Calculate heading error
        target_location = target_waypoint.transform.location
        heading_error = self._calculate_heading_error(
            vehicle_location, vehicle_yaw, target_location
        )

        # Calculate lateral error (cross-track error)
        lateral_error = self._calculate_lateral_error(
            vehicle_location, vehicle_yaw, current_waypoint
        )

        # Combined error (heading + lateral offset)
        total_error = heading_error + math.degrees(math.atan2(lateral_error, self.lookahead_distance))

        # PD control
        error_derivative = (total_error - self.previous_error) / dt if dt > 0 else 0.0
        steering = self.k_p * total_error + self.k_d * error_derivative

        self.previous_error = total_error

        # Normalize to [-1, 1]
        steering = clamp(steering / 45.0, -1.0, 1.0)

        return steering

    def _get_lane_type(self):
        """Get the lane type for waypoint queries."""
        try:
            import carla
            return carla.LaneType.Driving
        except ImportError:
            # Simulation mode - return mock lane type
            from simulation.mock_carla import LaneType
            return LaneType.Driving

    def _get_lookahead_waypoint(self, current_waypoint):
        """
        Get a waypoint at lookahead distance ahead.

        Args:
            current_waypoint: Current CARLA waypoint

        Returns:
            carla.Waypoint: Target waypoint ahead
        """
        # Get waypoints ahead
        next_waypoints = current_waypoint.next(self.lookahead_distance)

        if not next_waypoints:
            return None

        # Return the first waypoint (follows current lane)
        return next_waypoints[0]

    def _calculate_heading_error(self, vehicle_location, vehicle_yaw, target_location):
        """
        Calculate the heading error to the target.

        Args:
            vehicle_location: Current vehicle location
            vehicle_yaw: Current vehicle yaw in degrees
            target_location: Target location

        Returns:
            float: Heading error in degrees
        """
        # Calculate desired heading
        dx = target_location.x - vehicle_location.x
        dy = target_location.y - vehicle_location.y
        target_yaw = math.degrees(math.atan2(dy, dx))

        # Calculate error
        heading_error = normalize_angle(target_yaw - vehicle_yaw)

        return heading_error

    def _calculate_lateral_error(self, vehicle_location, vehicle_yaw, waypoint):
        """
        Calculate lateral (cross-track) error from lane center.

        Args:
            vehicle_location: Current vehicle location
            waypoint: Reference waypoint

        Returns:
            float: Lateral error in meters (positive = right of center)
        """
        waypoint_location = waypoint.transform.location
        waypoint_yaw = math.radians(waypoint.transform.rotation.yaw)

        # Vector from waypoint to vehicle
        dx = vehicle_location.x - waypoint_location.x
        dy = vehicle_location.y - waypoint_location.y

        # Project onto perpendicular axis (lateral direction)
        lateral_error = -dx * math.sin(waypoint_yaw) + dy * math.cos(waypoint_yaw)

        return lateral_error

    def get_current_lane_id(self):
        """
        Get current lane ID.

        Returns:
            int: Lane ID
        """
        vehicle_location = self.vehicle.get_transform().location
        waypoint = self.map.get_waypoint(vehicle_location, project_to_road=True)

        if waypoint:
            return waypoint.lane_id
        return None

    def is_in_lane(self, threshold=1.5):
        """
        Check if vehicle is within the lane.

        Args:
            threshold: Maximum lateral deviation in meters

        Returns:
            bool: True if vehicle is in lane
        """
        vehicle_location = self.vehicle.get_transform().location
        vehicle_yaw = self.vehicle.get_transform().rotation.yaw

        waypoint = self.map.get_waypoint(vehicle_location, project_to_road=True)

        if waypoint is None:
            return False

        lateral_error = abs(self._calculate_lateral_error(
            vehicle_location, vehicle_yaw, waypoint
        ))

        return lateral_error < threshold
