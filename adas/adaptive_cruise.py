"""
Adaptive Cruise Control Module

Maintains target speed while adapting to vehicles ahead.
Implements emergency braking when necessary.
"""

import math
from utils.helpers import get_speed, clamp
import config


class AdaptiveCruiseControl:
    """
    Adaptive Cruise Control (ACC) with emergency braking capability.
    """

    def __init__(self, vehicle):
        """
        Initialize the ACC controller.

        Args:
            vehicle: CARLA ego vehicle actor
        """
        self.vehicle = vehicle
        self.target_speed = config.TARGET_SPEED
        self.previous_speed_error = 0.0
        self.previous_distance_error = 0.0

        # PD controller gains
        self.k_p = config.ACC_K_P
        self.k_d = config.ACC_K_D

        # Safety parameters
        self.safe_distance = config.SAFE_DISTANCE
        self.time_gap = config.TIME_GAP
        self.emergency_brake_distance = config.EMERGENCY_BRAKE_DISTANCE
        self.emergency_brake_ttc = config.EMERGENCY_BRAKE_TTC

        # State
        self.is_emergency_braking = False

    def get_control(self, lead_vehicle_info, dt):
        """
        Calculate throttle and brake commands.

        Args:
            lead_vehicle_info: Dict with 'distance', 'relative_velocity' of lead vehicle
                              or None if no vehicle ahead
            dt: Time delta since last update

        Returns:
            tuple: (throttle, brake) values in range [0, 1]
        """
        current_speed = get_speed(self.vehicle)  # km/h
        current_speed_ms = current_speed / 3.6  # m/s

        # Check for emergency braking conditions
        if lead_vehicle_info is not None:
            distance = lead_vehicle_info['distance']
            relative_velocity = lead_vehicle_info.get('relative_velocity', 0)  # m/s, negative = approaching

            # Time to collision (TTC)
            if relative_velocity < -0.1:  # Approaching
                ttc = -distance / relative_velocity
            else:
                ttc = float('inf')

            # Emergency brake check
            if distance < self.emergency_brake_distance or ttc < self.emergency_brake_ttc:
                self.is_emergency_braking = True
                return 0.0, 1.0  # Full brake

        self.is_emergency_braking = False

        # Calculate desired speed based on lead vehicle
        if lead_vehicle_info is not None:
            distance = lead_vehicle_info['distance']
            desired_speed = self._calculate_desired_speed(distance, current_speed_ms)
        else:
            desired_speed = self.target_speed

        # Speed control
        speed_error = desired_speed - current_speed
        speed_error_derivative = (speed_error - self.previous_speed_error) / dt if dt > 0 else 0.0

        control_signal = self.k_p * speed_error + self.k_d * speed_error_derivative
        self.previous_speed_error = speed_error

        # Convert to throttle/brake
        if control_signal >= 0:
            throttle = clamp(control_signal / 10.0, 0.0, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = clamp(-control_signal / 10.0, 0.0, 1.0)

        return throttle, brake

    def _calculate_desired_speed(self, distance, current_speed_ms):
        """
        Calculate desired speed based on distance to lead vehicle.

        Args:
            distance: Distance to lead vehicle in meters
            current_speed_ms: Current speed in m/s

        Returns:
            float: Desired speed in km/h
        """
        # Calculate dynamic safe distance based on time gap
        dynamic_safe_distance = max(
            self.safe_distance,
            current_speed_ms * self.time_gap
        )

        if distance < self.safe_distance:
            # Too close - reduce speed significantly
            speed_factor = (distance / self.safe_distance) ** 2
            return self.target_speed * speed_factor
        elif distance < dynamic_safe_distance:
            # Within time gap - gradually reduce speed
            speed_factor = distance / dynamic_safe_distance
            return self.target_speed * speed_factor
        else:
            # Safe distance - maintain target speed
            return self.target_speed

    def set_target_speed(self, speed):
        """
        Set target cruising speed.

        Args:
            speed: Target speed in km/h
        """
        self.target_speed = clamp(speed, config.MIN_SPEED, config.MAX_SPEED)

    def increase_speed(self, increment=5.0):
        """Increase target speed by increment km/h."""
        self.set_target_speed(self.target_speed + increment)

    def decrease_speed(self, decrement=5.0):
        """Decrease target speed by decrement km/h."""
        self.set_target_speed(self.target_speed - decrement)

    def get_following_distance(self):
        """
        Get the current desired following distance.

        Returns:
            float: Following distance in meters
        """
        current_speed_ms = get_speed(self.vehicle) / 3.6
        return max(self.safe_distance, current_speed_ms * self.time_gap)

    def react_to_cutin(self, cutin_vehicle_info):
        """
        React to a vehicle cutting in.

        Args:
            cutin_vehicle_info: Dict with 'distance', 'relative_velocity' of cut-in vehicle

        Returns:
            tuple: (throttle, brake) for immediate response
        """
        if cutin_vehicle_info is None:
            return None

        distance = cutin_vehicle_info['distance']
        relative_velocity = cutin_vehicle_info.get('relative_velocity', 0)

        # Calculate TTC
        if relative_velocity < -0.1:
            ttc = -distance / relative_velocity
        else:
            ttc = float('inf')

        # Aggressive braking for cut-ins (more conservative than normal ACC)
        if distance < self.emergency_brake_distance * 1.5 or ttc < self.emergency_brake_ttc * 1.5:
            self.is_emergency_braking = True
            return 0.0, 1.0  # Full emergency brake
        elif distance < self.safe_distance * 1.5:
            # Moderate braking
            brake_intensity = 1.0 - (distance / (self.safe_distance * 1.5))
            return 0.0, clamp(brake_intensity, 0.3, 0.8)

        return None  # Let normal ACC handle it
