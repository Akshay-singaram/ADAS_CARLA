"""
Vehicle Detection Module

Detects vehicles ahead and cut-ins using CARLA world queries.
"""

import math
from utils.helpers import (
    get_relative_position,
    get_relative_velocity,
    calculate_distance_2d,
    get_speed
)
import config


class VehicleDetector:
    """
    Detects and tracks vehicles in the ego vehicle's vicinity.
    Specializes in detecting lead vehicles and cut-ins.
    """

    def __init__(self, vehicle, world, world_map):
        """
        Initialize the vehicle detector.

        Args:
            vehicle: CARLA ego vehicle actor
            world: CARLA world
            world_map: CARLA world map
        """
        self.vehicle = vehicle
        self.world = world
        self.map = world_map

        # Detection parameters
        self.detection_range = config.RADAR_RANGE
        self.fov = config.RADAR_FOV
        self.cutin_lane_threshold = config.CUTIN_LANE_THRESHOLD
        self.cutin_distance_threshold = config.CUTIN_DISTANCE_THRESHOLD
        self.cutin_lateral_velocity_threshold = config.CUTIN_LATERAL_VELOCITY_THRESHOLD

        # Tracking state
        self.tracked_vehicles = {}  # id -> vehicle info
        self.lead_vehicle_id = None
        self.cutin_candidates = {}  # id -> tracking info

    def update(self):
        """
        Update vehicle detection.

        Returns:
            dict: Detection results with 'lead_vehicle' and 'cutin_detected' info
        """
        ego_transform = self.vehicle.get_transform()
        ego_velocity = self.vehicle.get_velocity()
        ego_location = ego_transform.location
        ego_yaw = ego_transform.rotation.yaw

        # Get current lane
        ego_waypoint = self.map.get_waypoint(ego_location, project_to_road=True)
        ego_lane_id = ego_waypoint.lane_id if ego_waypoint else None

        # Find all vehicles in range
        all_vehicles = self.world.get_actors().filter('vehicle.*')

        lead_vehicle_info = None
        cutin_detected = None
        min_lead_distance = float('inf')

        for other_vehicle in all_vehicles:
            # Skip ego vehicle
            if other_vehicle.id == self.vehicle.id:
                continue

            other_location = other_vehicle.get_transform().location
            other_velocity = other_vehicle.get_velocity()

            # Get relative position
            longitudinal, lateral = get_relative_position(ego_transform, other_location)

            # Skip vehicles behind or too far
            if longitudinal < 0 or longitudinal > self.detection_range:
                continue

            # Check if in FOV
            angle = math.degrees(math.atan2(abs(lateral), longitudinal))
            if angle > self.fov / 2:
                continue

            # Get relative velocity
            rel_vel_long, rel_vel_lat = get_relative_velocity(
                ego_velocity, other_velocity, ego_yaw
            )

            # Get other vehicle's lane
            other_waypoint = self.map.get_waypoint(other_location, project_to_road=True)
            other_lane_id = other_waypoint.lane_id if other_waypoint else None

            # Check if this is a lead vehicle (same lane, ahead)
            is_same_lane = (ego_lane_id is not None and
                          other_lane_id is not None and
                          ego_lane_id == other_lane_id)

            if is_same_lane and abs(lateral) < 2.0:  # In same lane
                if longitudinal < min_lead_distance:
                    min_lead_distance = longitudinal
                    lead_vehicle_info = {
                        'id': other_vehicle.id,
                        'distance': longitudinal,
                        'lateral_offset': lateral,
                        'relative_velocity': rel_vel_long,
                        'vehicle': other_vehicle
                    }
                    self.lead_vehicle_id = other_vehicle.id

            # Check for cut-in
            cutin_info = self._detect_cutin(
                other_vehicle.id,
                longitudinal, lateral,
                rel_vel_long, rel_vel_lat,
                ego_lane_id, other_lane_id
            )

            if cutin_info is not None:
                if cutin_detected is None or cutin_info['distance'] < cutin_detected['distance']:
                    cutin_detected = cutin_info

        return {
            'lead_vehicle': lead_vehicle_info,
            'cutin_detected': cutin_detected
        }

    def _detect_cutin(self, vehicle_id, longitudinal, lateral, rel_vel_long, rel_vel_lat,
                      ego_lane_id, other_lane_id):
        """
        Detect if a vehicle is cutting into our lane.

        Args:
            vehicle_id: ID of the other vehicle
            longitudinal: Longitudinal distance
            lateral: Lateral distance
            rel_vel_long: Relative longitudinal velocity
            rel_vel_lat: Relative lateral velocity
            ego_lane_id: Ego vehicle's lane ID
            other_lane_id: Other vehicle's lane ID

        Returns:
            dict: Cut-in info or None
        """
        # Only check vehicles within cut-in detection range
        if longitudinal > self.cutin_distance_threshold:
            return None

        # Track lateral movement history
        if vehicle_id not in self.cutin_candidates:
            self.cutin_candidates[vehicle_id] = {
                'lateral_history': [lateral],
                'confirmed': False
            }
        else:
            self.cutin_candidates[vehicle_id]['lateral_history'].append(lateral)
            # Keep only recent history
            if len(self.cutin_candidates[vehicle_id]['lateral_history']) > 10:
                self.cutin_candidates[vehicle_id]['lateral_history'].pop(0)

        history = self.cutin_candidates[vehicle_id]['lateral_history']

        # Detect cut-in conditions:
        # 1. Vehicle is in adjacent lane (different lane ID)
        # 2. Vehicle is moving laterally toward our lane
        # 3. Vehicle is getting close to our lane

        is_adjacent_lane = (ego_lane_id is not None and
                          other_lane_id is not None and
                          ego_lane_id != other_lane_id)

        # Determine if vehicle is moving toward our lane
        moving_toward_us = False
        if lateral > 0:  # Vehicle is to our right
            moving_toward_us = rel_vel_lat < -self.cutin_lateral_velocity_threshold
        else:  # Vehicle is to our left
            moving_toward_us = rel_vel_lat > self.cutin_lateral_velocity_threshold

        # Check lateral movement trend from history
        if len(history) >= 3:
            lateral_trend = history[-1] - history[0]
            if lateral > 0:  # To our right
                trend_toward_us = lateral_trend < -0.5
            else:  # To our left
                trend_toward_us = lateral_trend > 0.5
        else:
            trend_toward_us = False

        # Detect cut-in
        is_cutin = (
            is_adjacent_lane and
            abs(lateral) < self.cutin_lane_threshold * 2 and  # Getting close
            (moving_toward_us or trend_toward_us) and
            longitudinal > 0 and longitudinal < self.cutin_distance_threshold
        )

        # Urgent cut-in: vehicle is almost in our lane
        is_urgent_cutin = (
            abs(lateral) < self.cutin_lane_threshold and
            longitudinal > 0 and longitudinal < self.cutin_distance_threshold * 0.5
        )

        if is_cutin or is_urgent_cutin:
            self.cutin_candidates[vehicle_id]['confirmed'] = True
            return {
                'id': vehicle_id,
                'distance': longitudinal,
                'lateral_offset': lateral,
                'relative_velocity': rel_vel_long,
                'lateral_velocity': rel_vel_lat,
                'is_urgent': is_urgent_cutin
            }

        return None

    def get_lead_vehicle(self):
        """
        Get info about the current lead vehicle.

        Returns:
            dict: Lead vehicle info or None
        """
        result = self.update()
        return result['lead_vehicle']

    def get_cutin(self):
        """
        Check for cut-in vehicles.

        Returns:
            dict: Cut-in vehicle info or None
        """
        result = self.update()
        return result['cutin_detected']

    def clear_tracking(self):
        """Clear all tracking data."""
        self.tracked_vehicles.clear()
        self.cutin_candidates.clear()
        self.lead_vehicle_id = None
