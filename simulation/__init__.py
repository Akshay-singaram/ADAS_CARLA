"""
ADAS Simulation Module

Provides lightweight simulation for testing ADAS without CARLA.
"""

from .mock_carla import (
    Vector3D, Location, Rotation, Transform,
    Waypoint, LaneType, VehicleControl
)
from .vehicle_sim import SimulatedVehicle, NPCVehicle
from .world import SimulatedWorld, SimulatedMap, SimulatedClient
from .scenarios import get_scenario
from .visualization import SimulationDisplay

__all__ = [
    'Vector3D', 'Location', 'Rotation', 'Transform',
    'Waypoint', 'LaneType', 'VehicleControl',
    'SimulatedVehicle', 'NPCVehicle',
    'SimulatedWorld', 'SimulatedMap', 'SimulatedClient',
    'get_scenario',
    'SimulationDisplay'
]
