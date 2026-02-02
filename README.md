# ADAS CARLA

Advanced Driver Assistance System (ADAS) implementation for the CARLA Simulator.

## Features

- **Lane Following**: Waypoint-based lane keeping with PD steering control
- **Adaptive Cruise Control (ACC)**: Speed regulation with dynamic following distance
- **Cut-in Detection & Reaction**: Detects vehicles cutting into the lane and triggers appropriate braking response
- **Emergency Braking**: Automatic emergency braking when collision is imminent

## Project Structure

```
ADAS_CARLA/
├── main.py              # Main entry point with demo
├── config.py            # Configuration parameters
├── requirements.txt     # Python dependencies
├── adas/
│   ├── __init__.py
│   ├── controller.py    # Main ADAS controller
│   ├── lane_following.py    # Lane following module
│   ├── adaptive_cruise.py   # Adaptive cruise control
│   └── vehicle_detection.py # Vehicle and cut-in detection
└── utils/
    ├── __init__.py
    └── helpers.py       # Utility functions
```

## Requirements

- Python 3.7+
- CARLA Simulator 0.9.x
- CARLA Python API
- pygame
- numpy

## Installation

1. Install CARLA Simulator from [CARLA Releases](https://github.com/carla-simulator/carla/releases)

2. Install the CARLA Python API:
   ```bash
   # Option 1: Using pip (if available for your CARLA version)
   pip install carla

   # Option 2: From CARLA installation
   cd /path/to/carla/PythonAPI/carla/dist
   pip install carla-0.9.x-py3.x-linux-x86_64.whl
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

1. Start the CARLA simulator:
   ```bash
   ./CarlaUE4.sh  # Linux
   # or
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
| ESC | Quit |

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

### Lane Following
Uses CARLA's waypoint system to get the road geometry. A PD controller calculates steering based on:
- Heading error to the lookahead waypoint
- Lateral offset from lane center

### Adaptive Cruise Control
Maintains the target speed while adapting to vehicles ahead:
- Calculates dynamic following distance based on current speed
- Reduces speed when approaching slower vehicles
- Triggers emergency braking when collision is imminent

### Cut-in Detection
Monitors adjacent lanes for vehicles that may cut in:
- Tracks lateral position and velocity of nearby vehicles
- Detects when a vehicle starts moving into the ego lane
- Triggers proactive braking response

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

## License

MIT License
