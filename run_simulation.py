#!/usr/bin/env python3
"""
ADAS Simulation Runner

Run the ADAS system in lightweight simulation mode (no CARLA required).

Controls:
    ESC     - Quit
    SPACE   - Toggle ADAS on/off
    UP/W    - Increase target speed
    DOWN/S  - Decrease target speed
    1       - Load lead vehicle scenario
    2       - Load cut-in scenario
    3       - Load slow vehicle scenario
    4       - Load traffic scenario
    5       - Load emergency brake scenario
"""

import sys
import time

# Add simulation module to use mock CARLA classes
from simulation import (
    SimulatedWorld, SimulatedClient, VehicleControl,
    SimulationDisplay, get_scenario
)
from simulation.mock_carla import Transform, Location, Rotation
from simulation.vehicle_sim import SimulatedVehicle

from adas import ADASController
import config


def main():
    """Main simulation function."""
    display = None
    current_scenario = None

    try:
        print("Starting ADAS Simulation (No CARLA Required)")
        print("=" * 50)

        # Create simulated world
        client = SimulatedClient()
        world = client.get_world()
        world_map = world.get_map()

        print(f"Simulated map: {world_map.name}")

        # Configure world
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / config.CONTROL_FREQUENCY
        world.apply_settings(settings)

        # Spawn ego vehicle
        print("Spawning ego vehicle...")
        spawn_point = Transform(
            Location(0, 0, 0),
            Rotation(0, 0, 0)
        )
        ego_vehicle = SimulatedVehicle(
            vehicle_id=1,
            x=0, y=0, yaw=0,
            speed=config.TARGET_SPEED / 3.6  # Convert km/h to m/s
        )
        ego_vehicle.lane_id = 0
        world.vehicles.append(ego_vehicle)
        world.set_ego_vehicle(ego_vehicle)

        print(f"Ego vehicle spawned at lane {ego_vehicle.lane_id}")

        # Initialize ADAS controller
        print("Initializing ADAS controller...")
        adas = ADASController(ego_vehicle, world, world_map)

        # Initialize display
        display = SimulationDisplay(width=1000, height=600)

        print("\nADAS Simulation Started!")
        print("-" * 50)
        print("Controls:")
        print("  SPACE     - Toggle ADAS on/off")
        print("  UP/W      - Increase target speed")
        print("  DOWN/S    - Decrease target speed")
        print("  1-5       - Load scenarios (lead/cutin/slow/traffic/emergency)")
        print("  ESC       - Quit")
        print("-" * 50)

        # Load default scenario
        print("\nLoading default lead vehicle scenario...")
        current_scenario = get_scenario('lead', world, lead_speed_kmh=40, distance_ahead=50)
        current_scenario.setup()

        # Main loop
        running = True
        frame_time = 1.0 / 60  # Target 60 FPS for display

        while running:
            loop_start = time.time()

            # Tick the simulation
            world.tick()
            timestamp = world.get_snapshot().timestamp

            # Handle input
            running, events = display.handle_events()

            if events['toggle_adas']:
                enabled = adas.toggle()
                print(f"ADAS {'ENABLED' if enabled else 'DISABLED'}")

            if events['increase_speed']:
                adas.increase_speed()
                print(f"Target speed: {adas.status['target_speed']:.1f} km/h")

            if events['decrease_speed']:
                adas.decrease_speed()
                print(f"Target speed: {adas.status['target_speed']:.1f} km/h")

            # Load scenario
            if events['scenario']:
                # Cleanup current scenario
                if current_scenario:
                    current_scenario.cleanup()

                scenario_name = events['scenario']
                print(f"\nLoading {scenario_name} scenario...")

                # Reset ego position
                ego_vehicle.x = 0
                ego_vehicle.y = 0
                ego_vehicle.speed = config.TARGET_SPEED / 3.6

                # Load new scenario
                scenario_params = {
                    'lead': {'lead_speed_kmh': 40, 'distance_ahead': 50},
                    'cutin': {'cutin_speed_kmh': 50, 'trigger_distance': 40},
                    'slow': {'slow_speed_kmh': 20, 'distance_ahead': 80},
                    'traffic': {'num_vehicles': 5},
                    'emergency': {'trigger_time': 5.0},
                }
                current_scenario = get_scenario(
                    scenario_name, world,
                    **scenario_params.get(scenario_name, {})
                )
                current_scenario.setup()

            # Update scenario
            if current_scenario:
                current_scenario.update(world.dt)

            # Update ADAS and apply control
            control = adas.update(timestamp)
            ego_vehicle.apply_control(control)

            # Update display
            display.update(world, ego_vehicle, adas.get_status(), adas.get_debug_info())

            # Frame rate limiting
            elapsed = time.time() - loop_start
            if elapsed < frame_time:
                time.sleep(frame_time - elapsed)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\nCleaning up...")

        if current_scenario:
            current_scenario.cleanup()

        if display:
            display.cleanup()

        print("Done.")


if __name__ == '__main__':
    main()
