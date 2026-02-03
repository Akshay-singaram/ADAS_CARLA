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
    L       - Toggle manual / lane-follow mode
    UP/DOWN - Increase / decrease target speed (auto mode)
    W/S     - Throttle / brake (manual mode, progressive)
    A/D     - Steer left / right (manual mode)

Requirements:
    - CARLA simulator running on localhost:2000
    - CARLA Python API installed
"""

import sys
import math
import time
import pygame
import carla

from adas import ADASController
from utils.helpers import get_speed
import config


class ADASDemoDisplay:
    """Simple pygame display for ADAS status."""

    def __init__(self, width=400, height=350):
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

        # Mode
        manual_mode = debug_info.get('manual_mode', False)
        mode_color = (255, 200, 0) if manual_mode else (0, 200, 255)
        self._draw_text(f'Mode: {"MANUAL" if manual_mode else "AUTO"}  [L]', y, mode_color)
        y += line_height

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
        y = self.height - 75
        self._draw_text('SPACE: ADAS | L: Manual | ESC: Quit', y, (150, 150, 150))
        y += line_height
        if manual_mode:
            self._draw_text('WASD: Drive | A/D: Steer', y, (150, 150, 150))
        else:
            self._draw_text('UP/DOWN: Target Speed', y, (150, 150, 150))

        pygame.display.flip()

    def _draw_text(self, text, y, color=(255, 255, 255)):
        surface = self.font.render(text, True, color)
        self.display.blit(surface, (20, y))

    def handle_events(self):
        """Handle pygame events. Returns (running, events_dict)."""
        events = {
            'toggle_adas': False,
            'toggle_manual': False,
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
                elif event.key == pygame.K_l:
                    events['toggle_manual'] = True
                elif event.key == pygame.K_UP:
                    events['increase_speed'] = True
                elif event.key == pygame.K_DOWN:
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

        client.load_world(config.CARLA_MAP)
        world = client.get_world()
        world_map = world.get_map()

        print(f'Connected to map: {world_map.name}')

        # Set synchronous mode for consistent simulation
        settings = world.get_settings()
        original_settings = settings
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / config.CONTROL_FREQUENCY
        world.apply_settings(settings)

        # Get spectator for camera follow
        spectator = world.get_spectator()

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

        # Manual control state
        manual_mode = False
        manual_throttle = 0.0
        manual_brake = 0.0
        manual_steer = 0.0
        THROTTLE_RAMP = 0.4   # per second
        BRAKE_RAMP = 0.6      # per second

        # Main loop
        target_dt = 1.0 / config.CONTROL_FREQUENCY
        running = True
        while running:
            loop_start = time.time()

            # Tick the world
            world.tick()
            timestamp = world.get_snapshot().timestamp

            # Follow ego vehicle with spectator camera
            ego_transform = ego_vehicle.get_transform()
            yaw_rad = math.radians(ego_transform.rotation.yaw)
            spectator.set_transform(carla.Transform(
                carla.Location(
                    x=ego_transform.location.x - 10 * math.cos(yaw_rad),
                    y=ego_transform.location.y - 10 * math.sin(yaw_rad),
                    z=ego_transform.location.z + 5
                ),
                carla.Rotation(yaw=ego_transform.rotation.yaw, pitch=-20)
            ))

            # Handle input
            running, events = display.handle_events()

            if events['toggle_adas']:
                enabled = adas.toggle()
                print(f'ADAS {"enabled" if enabled else "disabled"}')

            if events['toggle_manual']:
                manual_mode = not manual_mode
                manual_throttle = 0.0
                manual_brake = 0.0
                manual_steer = 0.0
                print(f'Mode: {"Manual" if manual_mode else "Lane Follow"}')

            if events['increase_speed']:
                adas.increase_speed()
                print(f'Target speed: {adas.status["target_speed"]:.1f} km/h')

            if events['decrease_speed']:
                adas.decrease_speed()
                print(f'Target speed: {adas.status["target_speed"]:.1f} km/h')

            # Control
            if manual_mode:
                keys = pygame.key.get_pressed()

                # Throttle (W) / Brake (S) — progressive ramp
                if keys[pygame.K_w]:
                    manual_throttle = min(1.0, manual_throttle + THROTTLE_RAMP * target_dt)
                    manual_brake = 0.0
                elif keys[pygame.K_s]:
                    manual_brake = min(1.0, manual_brake + BRAKE_RAMP * target_dt)
                    manual_throttle = 0.0
                else:
                    manual_throttle = 0.0
                    manual_brake = 0.0

                # Steering (A left / D right)
                if keys[pygame.K_a]:
                    manual_steer = -0.7
                elif keys[pygame.K_d]:
                    manual_steer = 0.7
                else:
                    manual_steer = 0.0

                control = carla.VehicleControl()
                control.throttle = manual_throttle
                control.brake = manual_brake
                control.steer = manual_steer
            else:
                control = adas.update(timestamp)

            ego_vehicle.apply_control(control)

            # Update display
            if manual_mode:
                debug_info = {
                    'enabled': False,
                    'manual_mode': True,
                    'current_speed': get_speed(ego_vehicle),
                    'target_speed': 0,
                    'lead_distance': None,
                    'following_distance': 0,
                    'cutin_warning': False,
                    'emergency_brake': False,
                    'in_lane': False,
                    'lane_id': None
                }
            else:
                debug_info = adas.get_debug_info()
                debug_info['manual_mode'] = False
            display.update(adas.get_status(), debug_info)

            # Rate-limit loop to CONTROL_FREQUENCY
            sleep_time = target_dt - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

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
