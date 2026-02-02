"""
Main ADAS Controller

Integrates lane following, adaptive cruise control, and cut-in detection
into a unified vehicle control system.
"""

import carla
from .lane_following import LaneFollower
from .adaptive_cruise import AdaptiveCruiseControl
from .vehicle_detection import VehicleDetector
from utils.helpers import get_speed
import config


class ADASController:
    """
    Main ADAS controller that coordinates all ADAS subsystems.
    """

    def __init__(self, vehicle, world, world_map):
        """
        Initialize the ADAS controller.

        Args:
            vehicle: CARLA ego vehicle actor
            world: CARLA world
            world_map: CARLA world map
        """
        self.vehicle = vehicle
        self.world = world
        self.map = world_map

        # Initialize subsystems
        self.lane_follower = LaneFollower(vehicle, world_map)
        self.acc = AdaptiveCruiseControl(vehicle)
        self.vehicle_detector = VehicleDetector(vehicle, world, world_map)

        # Control state
        self.enabled = True
        self.last_update_time = None

        # Status
        self.status = {
            'lane_following': True,
            'acc_active': True,
            'current_speed': 0.0,
            'target_speed': config.TARGET_SPEED,
            'lead_vehicle_distance': None,
            'cutin_warning': False,
            'emergency_braking': False
        }

    def update(self, timestamp):
        """
        Main control loop update.

        Args:
            timestamp: CARLA timestamp

        Returns:
            carla.VehicleControl: Vehicle control command
        """
        if not self.enabled:
            return carla.VehicleControl()

        # Calculate dt
        if self.last_update_time is None:
            dt = 1.0 / config.CONTROL_FREQUENCY
        else:
            dt = timestamp.elapsed_seconds - self.last_update_time
            if dt <= 0:
                dt = 1.0 / config.CONTROL_FREQUENCY

        self.last_update_time = timestamp.elapsed_seconds

        # Update vehicle detection
        detection_result = self.vehicle_detector.update()
        lead_vehicle = detection_result['lead_vehicle']
        cutin = detection_result['cutin_detected']

        # Update status
        self.status['current_speed'] = get_speed(self.vehicle)
        self.status['lead_vehicle_distance'] = lead_vehicle['distance'] if lead_vehicle else None
        self.status['cutin_warning'] = cutin is not None

        # Get steering from lane follower
        steering = self.lane_follower.get_steering(dt)

        # Handle cut-in with priority
        if cutin is not None:
            cutin_response = self.acc.react_to_cutin(cutin)
            if cutin_response is not None:
                throttle, brake = cutin_response
                self.status['emergency_braking'] = brake > 0.8
            else:
                # Use cut-in as lead vehicle for ACC
                throttle, brake = self.acc.get_control(cutin, dt)
                self.status['emergency_braking'] = self.acc.is_emergency_braking
        else:
            # Normal ACC operation
            throttle, brake = self.acc.get_control(lead_vehicle, dt)
            self.status['emergency_braking'] = self.acc.is_emergency_braking

        # Create control command
        control = carla.VehicleControl()
        control.steer = steering
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False
        control.manual_gear_shift = False

        return control

    def enable(self):
        """Enable ADAS control."""
        self.enabled = True
        self.last_update_time = None

    def disable(self):
        """Disable ADAS control."""
        self.enabled = False

    def toggle(self):
        """Toggle ADAS on/off."""
        if self.enabled:
            self.disable()
        else:
            self.enable()
        return self.enabled

    def set_target_speed(self, speed):
        """
        Set target cruise speed.

        Args:
            speed: Target speed in km/h
        """
        self.acc.set_target_speed(speed)
        self.status['target_speed'] = self.acc.target_speed

    def increase_speed(self, increment=5.0):
        """Increase target speed."""
        self.acc.increase_speed(increment)
        self.status['target_speed'] = self.acc.target_speed

    def decrease_speed(self, decrement=5.0):
        """Decrease target speed."""
        self.acc.decrease_speed(decrement)
        self.status['target_speed'] = self.acc.target_speed

    def get_status(self):
        """
        Get current ADAS status.

        Returns:
            dict: Current status information
        """
        return self.status.copy()

    def is_emergency_braking(self):
        """Check if emergency braking is active."""
        return self.status['emergency_braking']

    def get_debug_info(self):
        """
        Get detailed debug information.

        Returns:
            dict: Debug information
        """
        return {
            'enabled': self.enabled,
            'current_speed': self.status['current_speed'],
            'target_speed': self.status['target_speed'],
            'lead_distance': self.status['lead_vehicle_distance'],
            'following_distance': self.acc.get_following_distance(),
            'cutin_warning': self.status['cutin_warning'],
            'emergency_brake': self.status['emergency_braking'],
            'in_lane': self.lane_follower.is_in_lane(),
            'lane_id': self.lane_follower.get_current_lane_id()
        }
