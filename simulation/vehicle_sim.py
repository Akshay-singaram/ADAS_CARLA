"""
Vehicle Simulation

Simple vehicle dynamics simulation for testing ADAS without CARLA.
"""

import math
from .mock_carla import Location, Rotation, Transform, Vector3D, VehicleControl


class SimulatedVehicle:
    """
    Simulated vehicle with basic physics.
    """

    def __init__(self, vehicle_id, x=0, y=0, yaw=0, speed=0):
        self.id = vehicle_id
        self.type_id = 'vehicle.simulated.car'

        # State
        self.x = x
        self.y = y
        self.z = 0
        self.yaw = yaw  # degrees
        self.speed = speed  # m/s

        # Velocity components
        self.vx = speed * math.cos(math.radians(yaw))
        self.vy = speed * math.sin(math.radians(yaw))

        # Vehicle parameters
        self.max_speed = 50.0  # m/s (~180 km/h)
        self.max_acceleration = 5.0  # m/s^2
        self.max_deceleration = 10.0  # m/s^2
        self.max_steer_angle = 45.0  # degrees
        self.wheelbase = 2.5  # meters

        # Control state
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0

        # Lane info
        self.lane_id = 0
        self.target_lane_id = 0

    def get_transform(self):
        return Transform(
            Location(self.x, self.y, self.z),
            Rotation(0, self.yaw, 0)
        )

    def get_velocity(self):
        return Vector3D(self.vx, self.vy, 0)

    def get_location(self):
        return Location(self.x, self.y, self.z)

    def apply_control(self, control):
        """Apply vehicle control."""
        self.throttle = control.throttle
        self.brake = control.brake
        self.steer = control.steer

    def update(self, dt):
        """Update vehicle physics."""
        # Calculate acceleration
        if self.brake > 0:
            accel = -self.max_deceleration * self.brake
        else:
            accel = self.max_acceleration * self.throttle

        # Update speed
        self.speed += accel * dt
        self.speed = max(0, min(self.speed, self.max_speed))

        # Steering (bicycle model approximation)
        if abs(self.speed) > 0.1:
            steer_angle = self.steer * self.max_steer_angle
            turn_rate = (self.speed / self.wheelbase) * math.tan(math.radians(steer_angle))
            self.yaw += math.degrees(turn_rate * dt)
            self.yaw = self.yaw % 360

        # Update velocity components
        yaw_rad = math.radians(self.yaw)
        self.vx = self.speed * math.cos(yaw_rad)
        self.vy = self.speed * math.sin(yaw_rad)

        # Update position
        self.x += self.vx * dt
        self.y += self.vy * dt

    def get_speed_kmh(self):
        """Get speed in km/h."""
        return self.speed * 3.6

    def destroy(self):
        """Cleanup (no-op for simulation)."""
        pass


class NPCVehicle(SimulatedVehicle):
    """
    NPC vehicle with simple AI behavior.
    """

    def __init__(self, vehicle_id, x=0, y=0, yaw=0, speed=0):
        super().__init__(vehicle_id, x, y, yaw, speed)
        self.autopilot = False
        self.target_speed = speed  # m/s
        self.behavior = 'normal'  # 'normal', 'cutin', 'slow'

        # Cut-in parameters
        self.cutin_start_x = None
        self.cutin_target_lane = 0
        self.cutin_progress = 0

    def set_autopilot(self, enabled):
        self.autopilot = enabled

    def set_behavior(self, behavior, **kwargs):
        """Set NPC behavior."""
        self.behavior = behavior
        if behavior == 'cutin':
            self.cutin_start_x = kwargs.get('start_x', self.x + 30)
            self.cutin_target_lane = kwargs.get('target_lane', 0)

    def update(self, dt, ego_vehicle=None):
        """Update with AI behavior."""
        if self.autopilot:
            self._autopilot_control(dt, ego_vehicle)

        super().update(dt)

    def _autopilot_control(self, dt, ego_vehicle):
        """Simple autopilot logic."""
        # Speed control - maintain target speed
        speed_error = self.target_speed - self.speed
        if speed_error > 0:
            self.throttle = min(0.5, speed_error / 5.0)
            self.brake = 0
        else:
            self.throttle = 0
            self.brake = min(0.5, -speed_error / 5.0)

        # Behavior-specific control
        if self.behavior == 'cutin' and ego_vehicle:
            self._cutin_behavior(dt, ego_vehicle)
        elif self.behavior == 'slow':
            self.target_speed = 5.0  # Slow down
        else:
            self.steer = 0  # Go straight

    def _cutin_behavior(self, dt, ego_vehicle):
        """Execute cut-in maneuver."""
        if self.cutin_start_x is None:
            return

        # Start cut-in when reaching trigger point
        if self.x >= self.cutin_start_x and self.cutin_progress < 1.0:
            # Calculate lateral offset to target lane
            target_y = ego_vehicle.y  # Move to ego's lane
            y_error = target_y - self.y

            # Smooth lane change
            self.cutin_progress += dt * 0.5  # Takes ~2 seconds
            self.cutin_progress = min(1.0, self.cutin_progress)

            # Steer toward target lane
            steer_amount = y_error * 0.1
            self.steer = max(-0.3, min(0.3, steer_amount))

            # Update lane when mostly complete
            if self.cutin_progress > 0.8:
                self.lane_id = self.cutin_target_lane
        else:
            self.steer = 0
