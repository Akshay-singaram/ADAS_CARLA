"""
Visualization Module

Simple 2D pygame visualization for the ADAS simulation.
"""

import math
import pygame


class SimulationDisplay:
    """
    2D top-down visualization of the simulation.
    """

    def __init__(self, width=1000, height=600):
        pygame.init()
        pygame.font.init()

        self.width = width
        self.height = height
        self.display = pygame.display.set_mode((width, height))
        pygame.display.set_caption('ADAS Simulation')

        # Fonts
        self.font = pygame.font.SysFont('monospace', 14)
        self.font_large = pygame.font.SysFont('monospace', 18)
        self.font_title = pygame.font.SysFont('monospace', 24)

        # View settings
        self.pixels_per_meter = 8
        self.camera_x = 0
        self.camera_y = 0

        # Colors
        self.colors = {
            'background': (40, 40, 40),
            'road': (60, 60, 60),
            'lane_marking': (200, 200, 200),
            'ego': (0, 150, 255),
            'npc': (255, 180, 0),
            'npc_cutin': (255, 80, 80),
            'text': (255, 255, 255),
            'warning': (255, 100, 100),
            'safe': (100, 255, 100),
            'panel_bg': (30, 30, 30),
        }

        # Panel dimensions
        self.panel_width = 250
        self.road_area_width = width - self.panel_width

    def update(self, world, ego_vehicle, adas_status, debug_info):
        """
        Render the current simulation state.

        Args:
            world: SimulatedWorld instance
            ego_vehicle: Ego vehicle instance
            adas_status: ADAS status dict
            debug_info: ADAS debug info dict
        """
        self.display.fill(self.colors['background'])

        if ego_vehicle:
            # Center camera on ego vehicle
            self.camera_x = ego_vehicle.x
            self.camera_y = ego_vehicle.y

        # Draw road
        self._draw_road(world.map)

        # Draw vehicles
        for vehicle in world.vehicles:
            is_ego = vehicle == ego_vehicle
            self._draw_vehicle(vehicle, is_ego)

        # Draw detection visualization
        if ego_vehicle and debug_info:
            self._draw_detection_zone(ego_vehicle, debug_info)

        # Draw status panel
        self._draw_status_panel(adas_status, debug_info)

        pygame.display.flip()

    def _world_to_screen(self, x, y):
        """Convert world coordinates to screen coordinates."""
        screen_x = self.road_area_width // 2 + (x - self.camera_x) * self.pixels_per_meter
        screen_y = self.height // 2 - (y - self.camera_y) * self.pixels_per_meter
        return int(screen_x), int(screen_y)

    def _draw_road(self, road_map):
        """Draw the road surface and lane markings."""
        # Road surface
        road_top = self._world_to_screen(0, road_map.num_lanes * road_map.lane_width)[1]
        road_bottom = self._world_to_screen(0, -road_map.num_lanes * road_map.lane_width)[1]
        road_rect = pygame.Rect(0, road_top, self.road_area_width, road_bottom - road_top)
        pygame.draw.rect(self.display, self.colors['road'], road_rect)

        # Lane markings
        for lane in range(-road_map.num_lanes, road_map.num_lanes + 1):
            y = lane * road_map.lane_width - road_map.lane_width / 2
            _, screen_y = self._world_to_screen(0, y)

            # Draw dashed lines
            dash_length = 30
            gap_length = 20
            for x_start in range(-500, 500, dash_length + gap_length):
                x1, _ = self._world_to_screen(self.camera_x + x_start, 0)
                x2, _ = self._world_to_screen(self.camera_x + x_start + dash_length, 0)
                if 0 <= x1 <= self.road_area_width or 0 <= x2 <= self.road_area_width:
                    pygame.draw.line(self.display, self.colors['lane_marking'],
                                   (x1, screen_y), (x2, screen_y), 2)

    def _draw_vehicle(self, vehicle, is_ego=False):
        """Draw a vehicle."""
        x, y = self._world_to_screen(vehicle.x, vehicle.y)

        # Vehicle dimensions (in pixels)
        length = int(4.5 * self.pixels_per_meter)
        width = int(2.0 * self.pixels_per_meter)

        # Determine color
        if is_ego:
            color = self.colors['ego']
        elif hasattr(vehicle, 'behavior') and vehicle.behavior == 'cutin':
            color = self.colors['npc_cutin']
        else:
            color = self.colors['npc']

        # Create rotated rectangle
        angle = -vehicle.yaw  # Pygame y-axis is inverted
        self._draw_rotated_rect(x, y, length, width, angle, color)

        # Draw direction indicator
        yaw_rad = math.radians(vehicle.yaw)
        front_x = x + int(length/2 * math.cos(-yaw_rad))
        front_y = y + int(length/2 * math.sin(-yaw_rad))
        pygame.draw.circle(self.display, (255, 255, 255), (front_x, front_y), 4)

        # Speed label
        speed_kmh = vehicle.get_speed_kmh()
        label = self.font.render(f'{speed_kmh:.0f}', True, self.colors['text'])
        self.display.blit(label, (x - 10, y - 25))

    def _draw_rotated_rect(self, cx, cy, length, width, angle, color):
        """Draw a rotated rectangle."""
        angle_rad = math.radians(angle)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        # Calculate corners
        hw, hl = width / 2, length / 2
        corners = [
            (-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)
        ]

        rotated = []
        for px, py in corners:
            rx = px * cos_a - py * sin_a + cx
            ry = px * sin_a + py * cos_a + cy
            rotated.append((rx, ry))

        pygame.draw.polygon(self.display, color, rotated)
        pygame.draw.polygon(self.display, (255, 255, 255), rotated, 2)

    def _draw_detection_zone(self, ego_vehicle, debug_info):
        """Draw the vehicle detection zone."""
        ex, ey = self._world_to_screen(ego_vehicle.x, ego_vehicle.y)

        # Detection range arc
        import config
        range_pixels = int(config.RADAR_RANGE * self.pixels_per_meter)
        fov_half = config.RADAR_FOV / 2

        # Draw FOV cone
        yaw = -ego_vehicle.yaw
        start_angle = math.radians(yaw - fov_half)
        end_angle = math.radians(yaw + fov_half)

        points = [(ex, ey)]
        for angle in [start_angle, end_angle]:
            px = ex + range_pixels * math.cos(angle)
            py = ey + range_pixels * math.sin(angle)
            points.append((px, py))

        # Semi-transparent detection zone
        s = pygame.Surface((self.road_area_width, self.height), pygame.SRCALPHA)
        pygame.draw.polygon(s, (0, 255, 0, 30), points)
        self.display.blit(s, (0, 0))

        # Lead vehicle indicator
        lead_dist = debug_info.get('lead_distance')
        if lead_dist:
            yaw_rad = math.radians(-ego_vehicle.yaw)
            lead_x = ex + int(lead_dist * self.pixels_per_meter * math.cos(yaw_rad))
            lead_y = ey + int(lead_dist * self.pixels_per_meter * math.sin(yaw_rad))
            pygame.draw.circle(self.display, (255, 255, 0), (lead_x, lead_y), 8, 2)

    def _draw_status_panel(self, adas_status, debug_info):
        """Draw the status information panel."""
        panel_x = self.road_area_width
        panel_rect = pygame.Rect(panel_x, 0, self.panel_width, self.height)
        pygame.draw.rect(self.display, self.colors['panel_bg'], panel_rect)
        pygame.draw.line(self.display, (80, 80, 80),
                        (panel_x, 0), (panel_x, self.height), 2)

        x = panel_x + 15
        y = 20

        # Title
        title = self.font_title.render('ADAS Status', True, self.colors['text'])
        self.display.blit(title, (x, y))
        y += 40

        if not debug_info:
            return

        # ADAS enabled status
        enabled = debug_info.get('enabled', False)
        status_color = self.colors['safe'] if enabled else self.colors['warning']
        status_text = 'ENABLED' if enabled else 'DISABLED'
        self._draw_status_item(x, y, 'ADAS:', status_text, status_color)
        y += 30

        # Speed
        current_speed = debug_info.get('current_speed', 0)
        target_speed = debug_info.get('target_speed', 0)
        self._draw_status_item(x, y, 'Speed:', f'{current_speed:.1f} km/h')
        y += 25
        self._draw_status_item(x, y, 'Target:', f'{target_speed:.1f} km/h')
        y += 35

        # Lead vehicle
        lead_dist = debug_info.get('lead_distance')
        if lead_dist:
            self._draw_status_item(x, y, 'Lead Vehicle:', f'{lead_dist:.1f} m',
                                  self.colors['warning'])
        else:
            self._draw_status_item(x, y, 'Lead Vehicle:', 'None')
        y += 25

        # Following distance
        following = debug_info.get('following_distance', 0)
        self._draw_status_item(x, y, 'Safe Dist:', f'{following:.1f} m')
        y += 35

        # Lane status
        in_lane = debug_info.get('in_lane', False)
        lane_color = self.colors['safe'] if in_lane else self.colors['warning']
        self._draw_status_item(x, y, 'In Lane:', 'Yes' if in_lane else 'No', lane_color)
        y += 35

        # Warnings
        cutin = debug_info.get('cutin_warning', False)
        if cutin:
            warning_text = self.font_large.render('CUT-IN WARNING!', True,
                                                 self.colors['warning'])
            self.display.blit(warning_text, (x, y))
            y += 30

        emergency = debug_info.get('emergency_brake', False)
        if emergency:
            warning_text = self.font_large.render('EMERGENCY BRAKE!', True,
                                                 (255, 0, 0))
            self.display.blit(warning_text, (x, y))
            y += 30

        # Controls help
        y = self.height - 120
        pygame.draw.line(self.display, (80, 80, 80),
                        (panel_x + 10, y - 10), (panel_x + self.panel_width - 10, y - 10))

        controls = [
            'Controls:',
            'SPACE - Toggle ADAS',
            'UP/W - Increase speed',
            'DOWN/S - Decrease speed',
            '1-5 - Load scenario',
            'ESC - Quit'
        ]
        for line in controls:
            text = self.font.render(line, True, (150, 150, 150))
            self.display.blit(text, (x, y))
            y += 18

    def _draw_status_item(self, x, y, label, value, value_color=None):
        """Draw a status label and value."""
        if value_color is None:
            value_color = self.colors['text']

        label_surface = self.font.render(label, True, (180, 180, 180))
        value_surface = self.font.render(str(value), True, value_color)

        self.display.blit(label_surface, (x, y))
        self.display.blit(value_surface, (x + 100, y))

    def handle_events(self):
        """
        Handle pygame events.

        Returns:
            tuple: (running, events_dict)
        """
        events = {
            'toggle_adas': False,
            'increase_speed': False,
            'decrease_speed': False,
            'scenario': None,
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
                elif event.key == pygame.K_1:
                    events['scenario'] = 'lead'
                elif event.key == pygame.K_2:
                    events['scenario'] = 'cutin'
                elif event.key == pygame.K_3:
                    events['scenario'] = 'slow'
                elif event.key == pygame.K_4:
                    events['scenario'] = 'traffic'
                elif event.key == pygame.K_5:
                    events['scenario'] = 'emergency'

        return True, events

    def cleanup(self):
        """Clean up pygame."""
        pygame.quit()
