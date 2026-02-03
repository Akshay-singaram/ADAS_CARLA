# ADAS CARLA

Advanced Driver Assistance System (ADAS) implementation for the CARLA Simulator.

**Now includes a lightweight simulation mode - no CARLA required!**

## Features

- **Lane Following**: Waypoint-based lane keeping with PD steering control
- **Adaptive Cruise Control (ACC)**: Speed regulation with dynamic following distance
- **Cut-in Detection & Reaction**: Detects vehicles cutting into the lane and triggers appropriate braking response
- **Emergency Braking**: Automatic emergency braking when collision is imminent
- **Simulation Mode**: Test ADAS without CARLA (requires only pygame)

## Project Structure

```
ADAS_CARLA/
├── main.py              # CARLA mode entry point
├── run_simulation.py    # Simulation mode (no CARLA needed)
├── config.py            # Configuration parameters
├── requirements.txt     # Python dependencies
├── adas/
│   ├── __init__.py
│   ├── controller.py        # Main ADAS controller
│   ├── lane_following.py    # Lane following module
│   ├── adaptive_cruise.py   # Adaptive cruise control
│   └── vehicle_detection.py # Vehicle and cut-in detection
├── simulation/
│   ├── __init__.py
│   ├── mock_carla.py        # Mock CARLA classes
│   ├── vehicle_sim.py       # Vehicle physics simulation
│   ├── world.py             # Simulated world
│   ├── scenarios.py         # Test scenarios
│   └── visualization.py     # 2D pygame visualization
└── utils/
    ├── __init__.py
    ├── event_bus.py         # Synchronous pub/sub event bus
    └── helpers.py           # Utility functions
```

## Requirements

### Simulation Mode (Recommended for testing)
- Python 3.7+
- pygame
- numpy

### CARLA Mode
- Python 3.7+
- CARLA Simulator 0.9.x (requires 16GB+ RAM)
- CARLA Python API
- pygame
- numpy

## Installation

```bash
pip install -r requirements.txt
```

For CARLA mode, also install the CARLA Python API from [CARLA Releases](https://github.com/carla-simulator/carla/releases).

## Usage

### Simulation Mode (No CARLA Required)

Run the lightweight simulation with 2D visualization:

```bash
python run_simulation.py
```

This mode includes pre-built scenarios to test all ADAS features.

### CARLA Mode

1. Start the CARLA simulator:
   ```bash
   ./CarlaUE4.sh  # Linux
   CarlaUE4.exe   # Windows
   ```

2. Run the ADAS demo:
   ```bash
   python main.py
   ```

### Controls

| Key | Action |
|-----|--------|
| SPACE | Toggle ADAS on/off |
| UP / W | Increase target speed |
| DOWN / S | Decrease target speed |
| 1 | Load lead vehicle scenario |
| 2 | Load cut-in scenario |
| 3 | Load slow vehicle scenario |
| 4 | Load traffic scenario |
| 5 | Load emergency brake scenario |
| ESC | Quit |

### Test Scenarios (Simulation Mode)

1. **Lead Vehicle** (Key 1): A vehicle driving ahead at constant speed
2. **Cut-in** (Key 2): A vehicle from adjacent lane cuts in front
3. **Slow Vehicle** (Key 3): A slow-moving vehicle ahead
4. **Traffic** (Key 4): Multiple vehicles in various lanes
5. **Emergency Brake** (Key 5): Lead vehicle suddenly brakes

## Configuration

Edit `config.py` to adjust parameters:

### Vehicle Settings
- `TARGET_SPEED`: Default cruise speed (km/h)
- `MAX_SPEED`: Maximum allowed speed (km/h)

### Lane Following
- `LANE_FOLLOWING_K_P`: Proportional gain for steering
- `LANE_FOLLOWING_K_D`: Derivative gain for steering
- `LOOKAHEAD_DISTANCE`: Distance ahead to look for waypoints (m)

### Adaptive Cruise Control
- `SAFE_DISTANCE`: Minimum safe following distance (m)
- `TIME_GAP`: Time gap for following distance calculation (s)
- `EMERGENCY_BRAKE_DISTANCE`: Distance threshold for emergency braking (m)
- `EMERGENCY_BRAKE_TTC`: Time-to-collision threshold (s)

### Cut-in Detection
- `CUTIN_LANE_THRESHOLD`: Lateral threshold for cut-in detection (m)
- `CUTIN_DISTANCE_THRESHOLD`: Forward distance to monitor (m)

## How It Works

### Event-Bus Architecture

All ADAS subsystems communicate through a shared `EventBus` rather than direct method calls. The controller drives a two-phase event cycle every frame; each subsystem subscribes to the events it needs and publishes its outputs.

```
Controller.update()
  │
  ├─ publish('sensor_tick')
  │     └─> Detector  ──> publish('lead_vehicle_detected')  ──> ACC caches
  │                   ──> publish('cutin_detected')          ──> ACC caches
  │
  └─ publish('control_tick')
        ├─> LaneFollower ──> publish('steering_output')  ──> Controller caches
        └─> ACC          ──> publish('control_output')   ──> Controller caches
```

Speed-change commands (UP/DOWN keys) are also published as `target_speed_change` events rather than calling ACC directly. You can inspect the full event topology at runtime via `adas.event_bus.get_subscribers(event_name)`.

| Event | Published by | Subscribed by |
|-------|-------------|---------------|
| `sensor_tick` | Controller | Detector |
| `lead_vehicle_detected` | Detector | ACC |
| `cutin_detected` | Detector | ACC |
| `control_tick` | Controller | LaneFollower, ACC |
| `steering_output` | LaneFollower | Controller |
| `control_output` | ACC | Controller |
| `target_speed_change` | Controller | ACC |

### Lane Following
Uses CARLA's waypoint system to get the road geometry. A PD controller calculates steering based on:
- Heading error to the lookahead waypoint
- Lateral offset from lane center

Publishes its result as a `steering_output` event each frame.

### Adaptive Cruise Control
Maintains the target speed while adapting to vehicles ahead:
- Calculates dynamic following distance based on current speed
- Reduces speed when approaching slower vehicles
- Triggers emergency braking when collision is imminent
- Owns the cut-in priority logic: tries aggressive braking first, falls back to normal ACC if the cut-in is far enough

Subscribes to `lead_vehicle_detected` and `cutin_detected`; publishes `control_output`.

### Cut-in Detection
Monitors adjacent lanes for vehicles that may cut in:
- Tracks lateral position and velocity of nearby vehicles
- Detects when a vehicle starts moving into the ego lane
- Triggers proactive braking response

Runs during the sensor phase and publishes `cutin_detected` when a cut-in is confirmed.

## API Usage

You can use the ADAS modules in your own scripts:

```python
import carla
from adas import ADASController

# Connect to CARLA
client = carla.Client('localhost', 2000)
world = client.get_world()
world_map = world.get_map()

# Spawn or get your ego vehicle
ego_vehicle = ...

# Initialize ADAS
adas = ADASController(ego_vehicle, world, world_map)
adas.set_target_speed(50)  # km/h

# In your main loop
while True:
    world.tick()
    timestamp = world.get_snapshot().timestamp

    # Get control command
    control = adas.update(timestamp)
    ego_vehicle.apply_control(control)

    # Check status
    if adas.is_emergency_braking():
        print("Emergency braking!")
```
