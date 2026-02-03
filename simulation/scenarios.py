"""
Test Scenarios

Pre-defined scenarios for testing ADAS functionality.
"""

from .mock_carla import Transform, Location, Rotation
from .vehicle_sim import NPCVehicle


class Scenario:
    """Base scenario class."""

    def __init__(self, world):
        self.world = world
        self.npcs = []

    def setup(self):
        """Setup the scenario. Override in subclasses."""
        raise NotImplementedError

    def update(self, dt):
        """Update scenario state. Override if needed."""
        pass

    def cleanup(self):
        """Clean up scenario NPCs."""
        for npc in self.npcs:
            if npc in self.world.vehicles:
                self.world.vehicles.remove(npc)
        self.npcs.clear()


class LeadVehicleScenario(Scenario):
    """
    Simple scenario with a lead vehicle driving at constant speed.
    """

    def __init__(self, world, lead_speed_kmh=40, distance_ahead=50):
        super().__init__(world)
        self.lead_speed = lead_speed_kmh / 3.6  # Convert to m/s
        self.distance_ahead = distance_ahead

    def setup(self):
        ego = self.world.ego_vehicle
        if not ego:
            return

        # Spawn lead vehicle ahead in same lane
        lead = NPCVehicle(
            vehicle_id=100,
            x=ego.x + self.distance_ahead,
            y=ego.y,
            yaw=ego.yaw,
            speed=self.lead_speed
        )
        lead.lane_id = ego.lane_id
        lead.target_speed = self.lead_speed
        lead.set_autopilot(True)

        self.world.vehicles.append(lead)
        self.npcs.append(lead)

        return lead


class CutInScenario(Scenario):
    """
    Scenario where an NPC cuts in front of the ego vehicle.
    """

    def __init__(self, world, cutin_speed_kmh=50, trigger_distance=40):
        super().__init__(world)
        self.cutin_speed = cutin_speed_kmh / 3.6
        self.trigger_distance = trigger_distance

    def setup(self):
        ego = self.world.ego_vehicle
        if not ego:
            return

        # Spawn vehicle in adjacent lane, slightly ahead
        lane_offset = self.world.map.lane_width  # One lane to the right
        cutin_vehicle = NPCVehicle(
            vehicle_id=101,
            x=ego.x + 20,  # Start 20m ahead
            y=ego.y + lane_offset,
            yaw=ego.yaw,
            speed=self.cutin_speed
        )
        cutin_vehicle.lane_id = ego.lane_id + 1  # Adjacent lane
        cutin_vehicle.target_speed = self.cutin_speed
        cutin_vehicle.set_autopilot(True)
        cutin_vehicle.set_behavior('cutin',
            start_x=ego.x + self.trigger_distance,
            target_lane=ego.lane_id
        )

        self.world.vehicles.append(cutin_vehicle)
        self.npcs.append(cutin_vehicle)

        return cutin_vehicle


class SlowVehicleScenario(Scenario):
    """
    Scenario with a slow-moving vehicle ahead.
    """

    def __init__(self, world, slow_speed_kmh=20, distance_ahead=80):
        super().__init__(world)
        self.slow_speed = slow_speed_kmh / 3.6
        self.distance_ahead = distance_ahead

    def setup(self):
        ego = self.world.ego_vehicle
        if not ego:
            return

        # Spawn slow vehicle ahead
        slow_vehicle = NPCVehicle(
            vehicle_id=102,
            x=ego.x + self.distance_ahead,
            y=ego.y,
            yaw=ego.yaw,
            speed=self.slow_speed
        )
        slow_vehicle.lane_id = ego.lane_id
        slow_vehicle.target_speed = self.slow_speed
        slow_vehicle.set_autopilot(True)

        self.world.vehicles.append(slow_vehicle)
        self.npcs.append(slow_vehicle)

        return slow_vehicle


class TrafficScenario(Scenario):
    """
    Scenario with multiple traffic vehicles.
    """

    def __init__(self, world, num_vehicles=5):
        super().__init__(world)
        self.num_vehicles = num_vehicles

    def setup(self):
        ego = self.world.ego_vehicle
        if not ego:
            return

        lane_width = self.world.map.lane_width
        spawned = []

        # Spawn vehicles at various positions
        positions = [
            (50, 0, 45),      # Ahead, same lane
            (30, lane_width, 50),   # Ahead right, faster
            (70, -lane_width, 40),  # Ahead left, slower
            (-20, lane_width, 45),  # Behind right
            (100, 0, 35),     # Far ahead, slow
        ]

        for i, (dx, dy, speed_kmh) in enumerate(positions[:self.num_vehicles]):
            npc = NPCVehicle(
                vehicle_id=200 + i,
                x=ego.x + dx,
                y=ego.y + dy,
                yaw=ego.yaw,
                speed=speed_kmh / 3.6
            )
            npc.lane_id = self.world.map._get_lane_id(ego.y + dy)
            npc.target_speed = speed_kmh / 3.6
            npc.set_autopilot(True)

            self.world.vehicles.append(npc)
            self.npcs.append(npc)
            spawned.append(npc)

        return spawned


class EmergencyBrakeScenario(Scenario):
    """
    Scenario where lead vehicle suddenly brakes.
    """

    def __init__(self, world, trigger_time=5.0):
        super().__init__(world)
        self.trigger_time = trigger_time
        self.triggered = False
        self.lead_vehicle = None

    def setup(self):
        ego = self.world.ego_vehicle
        if not ego:
            return

        # Spawn lead vehicle at same speed
        self.lead_vehicle = NPCVehicle(
            vehicle_id=103,
            x=ego.x + 40,
            y=ego.y,
            yaw=ego.yaw,
            speed=ego.speed if ego.speed > 0 else 15
        )
        self.lead_vehicle.lane_id = ego.lane_id
        self.lead_vehicle.target_speed = self.lead_vehicle.speed
        self.lead_vehicle.set_autopilot(True)

        self.world.vehicles.append(self.lead_vehicle)
        self.npcs.append(self.lead_vehicle)

        return self.lead_vehicle

    def update(self, dt):
        """Check if it's time to trigger emergency brake."""
        if not self.triggered and self.world.elapsed_time >= self.trigger_time:
            if self.lead_vehicle:
                self.lead_vehicle.target_speed = 0
                self.lead_vehicle.set_behavior('slow')
                self.triggered = True


def get_scenario(name, world, **kwargs):
    """
    Factory function to get scenario by name.

    Args:
        name: Scenario name ('lead', 'cutin', 'slow', 'traffic', 'emergency')
        world: SimulatedWorld instance
        **kwargs: Additional scenario parameters

    Returns:
        Scenario instance
    """
    scenarios = {
        'lead': LeadVehicleScenario,
        'cutin': CutInScenario,
        'slow': SlowVehicleScenario,
        'traffic': TrafficScenario,
        'emergency': EmergencyBrakeScenario,
    }

    if name not in scenarios:
        raise ValueError(f"Unknown scenario: {name}. Available: {list(scenarios.keys())}")

    return scenarios[name](world, **kwargs)
