#!/usr/bin/env python3
"""
ADAS CARLA - Advanced Driver Assistance System Demo

This script demonstrates the ADAS system in CARLA simulator with:
- Lane Following
- Adaptive Cruise Control
- Cut-in Detection and Reaction

Controls:
    ESC     - Quit
    SPACE   - Toggle ADAS on/off
    UP/W    - Increase target speed
    DOWN/S  - Decrease target speed

Requirements:
    - CARLA simulator running on localhost:2000
    - CARLA Python API installed
"""

import sys
import time
import pygame
import carla

from adas import ADASController
from utils.helpers import get_speed
import config


class ADASDemoDisplay:
    """Simple pygame display for ADAS status."""

    def __init__(self, width=400, height=300):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode((width, height))
        pygame.display.set_caption('ADAS Control Panel')
        self.font = pygame.font.SysFont('monospace', 16)
        self.font_large = pygame.font.SysFont('monospace', 24)
        self.width = width
        self.height = height

    def update(self, adas_status, debug_info):
        """Update the display with current status."""
        self.display.fill((30, 30, 30))

        y = 20
        line_height = 25

        # Title
        title = self.font_large.render('ADAS Status', True, (255, 255, 255))
        self.display.blit(title, (self.width // 2 - title.get_width() // 2, y))
        y += 40

        # Status items
        enabled = debug_info.get('enabled', False)
        status_color = (0, 255, 0) if enabled else (255, 100, 100)
        status_text = 'ENABLED' if enabled else 'DISABLED'
        self._draw_text(f'ADAS: {status_text}', y, status_color)
        y += line_height

        # Speed
        current_speed = debug_info.get('current_speed', 0)
        target_speed = debug_info.get('target_speed', 0)
        self._draw_text(f'Speed: {current_speed:.1f} / {target_speed:.1f} km/h', y)
        y += line_height

        # Lead vehicle
        lead_distance = debug_info.get('lead_distance')
        if lead_distance:
            self._draw_text(f'Lead Vehicle: {lead_distance:.1f} m', y, (255, 255, 0))
        else:
            self._draw_text('Lead Vehicle: None', y)
        y += line_height

        # Following distance
        following = debug_info.get('following_distance', 0)
        self._draw_text(f'Following Distance: {following:.1f} m', y)
        y += line_height

        # Lane status
        in_lane = debug_info.get('in_lane', False)
        lane_color = (0, 255, 0) if in_lane else (255, 100, 100)
        self._draw_text(f'In Lane: {"Yes" if in_lane else "No"}', y, lane_color)
        y += line_height

        # Cut-in warning
        cutin = debug_info.get('cutin_warning', False)
        if cutin:
            self._draw_text('CUT-IN WARNING!', y, (255, 0, 0))
            y += line_height

        # Emergency brake
        emergency = debug_info.get('emergency_brake', False)
        if emergency:
            self._draw_text('EMERGENCY BRAKE!', y, (255, 0, 0))
            y += line_height

        # Controls hint
        y = self.height - 60
        self._draw_text('SPACE: Toggle | UP/DOWN: Speed', y, (150, 150, 150))
        y += line_height
        self._draw_text('ESC: Quit', y, (150, 150, 150))

        pygame.display.flip()

    def _draw_text(self, text, y, color=(255, 255, 255)):
        surface = self.font.render(text, True, color)
        self.display.blit(surface, (20, y))

    def handle_events(self):
        """Handle pygame events. Returns (running, events_dict)."""
        events = {
            'toggle_adas': False,
            'increase_speed': False,
            'decrease_speed': False
        }

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False, events
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False, events
                elif event.key == pygame.K_SPACE:
                    events['toggle_adas'] = True
                elif event.key in (pygame.K_UP, pygame.K_w):
                    events['increase_speed'] = True
                elif event.key in (pygame.K_DOWN, pygame.K_s):
                    events['decrease_speed'] = True

        return True, events

    def cleanup(self):
        pygame.quit()


def spawn_ego_vehicle(world, spawn_point=None):
    """Spawn the ego vehicle."""
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

    if spawn_point is None:
        spawn_points = world.get_map().get_spawn_points()
        spawn_point = spawn_points[0] if spawn_points else carla.Transform()

    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    return vehicle


def spawn_traffic(world, num_vehicles=10):
    """Spawn NPC traffic vehicles."""
    blueprint_library = world.get_blueprint_library()
    vehicle_blueprints = blueprint_library.filter('vehicle.*')

    spawn_points = world.get_map().get_spawn_points()
    spawned = []

    for i, spawn_point in enumerate(spawn_points[1:num_vehicles + 1]):
        bp = vehicle_blueprints[i % len(vehicle_blueprints)]
        try:
            vehicle = world.spawn_actor(bp, spawn_point)
            vehicle.set_autopilot(True)
            spawned.append(vehicle)
        except Exception:
            pass

    return spawned


def main():
    """Main function."""
    client = None
    ego_vehicle = None
    traffic_vehicles = []
    display = None

    try:
        # Connect to CARLA
        print(f'Connecting to CARLA at {config.CARLA_HOST}:{config.CARLA_PORT}...')
        client = carla.Client(config.CARLA_HOST, config.CARLA_PORT)
        client.set_timeout(config.CARLA_TIMEOUT)

        world = client.get_world()
        world_map = world.get_map()

        print(f'Connected to map: {world_map.name}')

        # Set synchronous mode for consistent simulation
        settings = world.get_settings()
        original_settings = settings
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / config.CONTROL_FREQUENCY
        world.apply_settings(settings)

        # Spawn ego vehicle
        print('Spawning ego vehicle...')
        ego_vehicle = spawn_ego_vehicle(world)
        print(f'Ego vehicle spawned: {ego_vehicle.type_id}')

        # Spawn traffic
        print('Spawning traffic...')
        traffic_vehicles = spawn_traffic(world, num_vehicles=15)
        print(f'Spawned {len(traffic_vehicles)} traffic vehicles')

        # Initialize ADAS controller
        print('Initializing ADAS controller...')
        adas = ADASController(ego_vehicle, world, world_map)

        # Initialize display
        display = ADASDemoDisplay()

        print('ADAS Demo started! Press SPACE to toggle, ESC to quit.')

        # Main loop
        running = True
        while running:
            # Tick the world
            world.tick()
            timestamp = world.get_snapshot().timestamp

            # Handle input
            running, events = display.handle_events()

            if events['toggle_adas']:
                enabled = adas.toggle()
                print(f'ADAS {"enabled" if enabled else "disabled"}')

            if events['increase_speed']:
                adas.increase_speed()
                print(f'Target speed: {adas.status["target_speed"]:.1f} km/h')

            if events['decrease_speed']:
                adas.decrease_speed()
                print(f'Target speed: {adas.status["target_speed"]:.1f} km/h')

            # Update ADAS and apply control
            control = adas.update(timestamp)
            ego_vehicle.apply_control(control)

            # Update display
            display.update(adas.get_status(), adas.get_debug_info())

    except KeyboardInterrupt:
        print('\nInterrupted by user.')

    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()

    finally:
        print('Cleaning up...')

        # Cleanup display
        if display:
            display.cleanup()

        # Destroy actors
        if ego_vehicle:
            ego_vehicle.destroy()

        for vehicle in traffic_vehicles:
            try:
                vehicle.destroy()
            except Exception:
                pass

        # Restore world settings
        if client:
            try:
                world = client.get_world()
                settings = world.get_settings()
                settings.synchronous_mode = False
                world.apply_settings(settings)
            except Exception:
                pass

        print('Done.')


if __name__ == '__main__':
    main()
