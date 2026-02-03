"""
Simulation World

Manages the simulated environment including roads, vehicles, and updates.
"""

import math
from .mock_carla import (
    Location, Rotation, Transform, Waypoint, LaneType,
    Timestamp, WorldSnapshot
)
from .vehicle_sim import SimulatedVehicle, NPCVehicle


class SimulatedMap:
    """
    Simple road map for simulation.
    Simulates a straight multi-lane highway.
    """

    def __init__(self):
        self.name = "SimulatedHighway"
        self.lane_width = 3.5  # meters
        self.num_lanes = 3
        self.road_length = 10000  # meters

    def get_waypoint(self, location, project_to_road=True, lane_type=None):
        """Get waypoint at location."""
        # Determine lane from y position
        lane_id = self._get_lane_id(location.y)

        # Project to lane center if requested
        if project_to_road:
            center_y = self._get_lane_center(lane_id)
        else:
            center_y = location.y

        transform = Transform(
            Location(location.x, center_y, 0),
            Rotation(0, 0, 0)  # Straight road, yaw = 0
        )

        return Waypoint(transform, lane_id, road_id=0)

    def get_spawn_points(self):
        """Get spawn points for vehicles."""
        points = []
        for lane in range(self.num_lanes):
            y = self._get_lane_center(lane)
            for x in range(0, 500, 50):
                points.append(Transform(
                    Location(x, y, 0),
                    Rotation(0, 0, 0)
                ))
        return points

    def _get_lane_id(self, y):
        """Determine lane ID from y position."""
        # Lane 0 is center, negative lanes to left, positive to right
        lane = round(y / self.lane_width)
        return max(-1, min(1, lane))  # Clamp to -1, 0, 1

    def _get_lane_center(self, lane_id):
        """Get y coordinate of lane center."""
        return lane_id * self.lane_width


class ActorList:
    """Mock actor list."""
    def __init__(self, actors):
        self._actors = actors

    def filter(self, pattern):
        """Filter actors by type pattern."""
        if 'vehicle' in pattern:
            return [a for a in self._actors if 'vehicle' in a.type_id]
        return self._actors

    def __iter__(self):
        return iter(self._actors)

    def __len__(self):
        return len(self._actors)


class SimulatedWorld:
    """
    Simulated world environment.
    """

    def __init__(self):
        self.map = SimulatedMap()
        self.vehicles = []
        self.ego_vehicle = None

        # Simulation state
        self.elapsed_time = 0.0
        self.dt = 0.05  # 20 Hz
        self.frame = 0

        # Settings
        self._synchronous = False

    def get_map(self):
        return self.map

    def get_actors(self):
        return ActorList(self.vehicles)

    def spawn_actor(self, blueprint, transform):
        """Spawn a vehicle."""
        vehicle_id = len(self.vehicles) + 1

        if 'npc' in str(blueprint).lower():
            vehicle = NPCVehicle(
                vehicle_id,
                transform.location.x,
                transform.location.y,
                transform.rotation.yaw,
                speed=0
            )
        else:
            vehicle = SimulatedVehicle(
                vehicle_id,
                transform.location.x,
                transform.location.y,
                transform.rotation.yaw,
                speed=0
            )

        # Set lane based on position
        vehicle.lane_id = self.map._get_lane_id(transform.location.y)

        self.vehicles.append(vehicle)
        return vehicle

    def tick(self):
        """Advance simulation by one step."""
        self.frame += 1
        self.elapsed_time += self.dt

        # Update all NPC vehicles
        for vehicle in self.vehicles:
            if isinstance(vehicle, NPCVehicle) and vehicle.autopilot:
                vehicle.update(self.dt, self.ego_vehicle)
            elif vehicle != self.ego_vehicle:
                # Non-autopilot NPCs just continue at current speed
                if isinstance(vehicle, NPCVehicle):
                    vehicle.update(self.dt)

        # Update ego vehicle (control already applied)
        if self.ego_vehicle:
            self.ego_vehicle.update(self.dt)

    def get_snapshot(self):
        """Get current world snapshot."""
        timestamp = Timestamp(self.elapsed_time)
        timestamp.frame = self.frame
        timestamp.delta_seconds = self.dt
        return WorldSnapshot(timestamp)

    def get_settings(self):
        """Get world settings."""
        return WorldSettings(self._synchronous, self.dt)

    def apply_settings(self, settings):
        """Apply world settings."""
        self._synchronous = settings.synchronous_mode
        if settings.fixed_delta_seconds:
            self.dt = settings.fixed_delta_seconds

    def set_ego_vehicle(self, vehicle):
        """Set the ego vehicle for reference."""
        self.ego_vehicle = vehicle

    def destroy_actors(self):
        """Clean up all actors."""
        for v in self.vehicles:
            v.destroy()
        self.vehicles.clear()
        self.ego_vehicle = None


class WorldSettings:
    """Mock world settings."""
    def __init__(self, sync=False, dt=0.05):
        self.synchronous_mode = sync
        self.fixed_delta_seconds = dt


class BlueprintLibrary:
    """Mock blueprint library."""
    def filter(self, pattern):
        return [f'vehicle.{pattern}']


class SimulatedClient:
    """
    Simulated CARLA client.
    """

    def __init__(self, host='localhost', port=2000):
        self.host = host
        self.port = port
        self.world = SimulatedWorld()

    def set_timeout(self, timeout):
        pass

    def get_world(self):
        return self.world
