"""
Main ADAS Controller

Integrates lane following, adaptive cruise control, and cut-in detection
into a unified vehicle control system.

Works with both real CARLA and simulation mode.
"""

try:
    import carla
except ImportError:
    # Use simulation mock when CARLA is not available
    from simulation.mock_carla import VehicleControl
    carla = type('carla', (), {'VehicleControl': VehicleControl})()

from .lane_following import LaneFollower
from .adaptive_cruise import AdaptiveCruiseControl
from .vehicle_detection import VehicleDetector
from utils.helpers import get_speed
from utils.event_bus import EventBus
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

        # Shared event bus — all subsystems publish/subscribe through this
        self.event_bus = EventBus()

        # Initialize subsystems (order matters: LaneFollower before ACC so
        # its control_tick subscription fires first)
        self.vehicle_detector = VehicleDetector(vehicle, world, world_map, self.event_bus)
        self.lane_follower = LaneFollower(vehicle, world_map, self.event_bus)
        self.acc = AdaptiveCruiseControl(vehicle, self.event_bus)

        # Subscribe to subsystem output events
        self.event_bus.subscribe('steering_output', self._on_steering_output, owner='Controller')
        self.event_bus.subscribe('control_output', self._on_control_output, owner='Controller')

        # Cached outputs — populated each frame by the event callbacks above
        self._latest_steering_output = {'steering': 0.0, 'lane_id': None, 'is_in_lane': True}
        self._latest_control_output = {
            'throttle': 0.0, 'brake': 0.0,
            'is_emergency_braking': False, 'following_distance': 0.0,
            'lead_vehicle_distance': None, 'cutin_active': False
        }

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

    # ------------------------------------------------------------------
    # Event callbacks (called synchronously during publish)
    # ------------------------------------------------------------------

    def _on_steering_output(self, payload):
        """Cache steering result. Subscribed to: steering_output"""
        self._latest_steering_output = payload

    def _on_control_output(self, payload):
        """Cache throttle/brake result. Subscribed to: control_output"""
        self._latest_control_output = payload

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def update(self, timestamp):
        """
        Main control loop update.  Drives the two-phase event cycle:
            Phase 1 — sensor_tick  →  Detector scans, publishes detections
            Phase 2 — control_tick →  LaneFollower & ACC compute, publish outputs
        Then assembles VehicleControl from the cached outputs.

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

        # Phase 1: sensor tick — Detector runs and publishes detection events
        self.event_bus.publish('sensor_tick', {'dt': dt})

        # Phase 2: control tick — LaneFollower then ACC run and publish outputs
        self.event_bus.publish('control_tick', {'dt': dt})

        # Assemble VehicleControl from cached event outputs
        control = carla.VehicleControl()
        control.steer = self._latest_steering_output['steering']
        control.throttle = self._latest_control_output['throttle']
        control.brake = self._latest_control_output['brake']
        control.hand_brake = False
        control.manual_gear_shift = False

        # Update status from cached outputs
        self.status['current_speed'] = get_speed(self.vehicle)
        self.status['lead_vehicle_distance'] = self._latest_control_output['lead_vehicle_distance']
        self.status['cutin_warning'] = self._latest_control_output['cutin_active']
        self.status['emergency_braking'] = self._latest_control_output['is_emergency_braking']

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
        self.event_bus.publish('target_speed_change', {'speed': speed})
        self.status['target_speed'] = self.acc.target_speed

    def increase_speed(self, increment=5.0):
        """Increase target speed."""
        self.event_bus.publish('target_speed_change', {'speed': self.acc.target_speed + increment})
        self.status['target_speed'] = self.acc.target_speed

    def decrease_speed(self, decrement=5.0):
        """Decrease target speed."""
        self.event_bus.publish('target_speed_change', {'speed': self.acc.target_speed - decrement})
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
        Get detailed debug information (all values from cached event outputs).

        Returns:
            dict: Debug information
        """
        return {
            'enabled': self.enabled,
            'current_speed': self.status['current_speed'],
            'target_speed': self.status['target_speed'],
            'lead_distance': self._latest_control_output['lead_vehicle_distance'],
            'following_distance': self._latest_control_output['following_distance'],
            'cutin_warning': self._latest_control_output['cutin_active'],
            'emergency_brake': self._latest_control_output['is_emergency_braking'],
            'in_lane': self._latest_steering_output['is_in_lane'],
            'lane_id': self._latest_steering_output['lane_id']
        }
