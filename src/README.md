# Space Arm ROS2 Simulation

ROS2 Humble-based simulation of a 6-DOF robotic arm for space operations, featuring MPC control, visual servoing, and a microgravity Gazebo environment. Designed to run entirely in GitHub Codespaces.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Gazebo Simulation                     │
│  ┌──────────┐  ┌──────────────┐  ┌───────────────────┐ │
│  │ Space Arm │  │ Camera Sensor│  │ Space Environment │ │
│  │  (URDF)   │  │  (640x480)   │  │  (gravity = 0)    │ │
│  └─────┬─────┘  └──────┬───────┘  └───────────────────┘ │
└────────┼───────────────┼────────────────────────────────┘
         │               │
    joint_states    image_raw
         │               │
    ┌────▼────┐     ┌────▼──────────┐
    │   MPC   │     │ Target        │
    │Controller│    │ Detector      │
    │(CasADi) │     │ (ArUco/Color) │
    └────┬────┘     └────┬──────────┘
         │               │
    effort_cmds     detected_pose
         │               │
         │          ┌────▼──────────┐
         │          │ Visual        │
         │          │ Servoing      │
         │          │ (IBVS)        │
         │          └───────────────┘
         │
    ┌────▼────────────┐
    │  ros2_control    │
    │  (effort iface)  │
    └─────────────────┘
```

## Packages

| Package | Description |
|---------|-------------|
| `space_arm_description` | URDF/xacro model of the 6-DOF arm with end-effector camera |
| `space_arm_control` | CasADi-based MPC controller for joint-space control |
| `space_arm_vision` | ArUco detection, visual servoing (IBVS) pipeline |
| `space_arm_simulation` | Gazebo world (microgravity), launch files, models |

## Quick Start (GitHub Codespaces)

### 1. Create Codespace

Click **Code > Codespaces > Create codespace on main** from this repository. The devcontainer will automatically install ROS2 Humble, MoveIt2, Gazebo, and all Python dependencies.

### 2. Access the Desktop

After the Codespace builds, open the **noVNC** desktop:
- Go to the **Ports** tab in VS Code
- Click the globe icon next to port **6080**
- Password: `vscode`

### 3. Build the Workspace

```bash
cd /workspaces/<repo-name>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch the Simulation

**Full simulation** (Gazebo + arm + MPC + vision):
```bash
ros2 launch space_arm_simulation full_simulation.launch.py
```

**Headless mode** (no Gazebo GUI, lighter on resources):
```bash
ros2 launch space_arm_simulation full_simulation.launch.py headless:=true
```

**View the arm in RViz only** (no physics):
```bash
ros2 launch space_arm_description display.launch.py
```

### 5. Send Commands

Send the arm through demo waypoints:
```bash
ros2 run space_arm_control joint_space_mpc.py
```

Publish a custom joint target:
```bash
ros2 topic pub /space_arm/target_joint_state sensor_msgs/msg/JointState \
  "{name: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], \
    position: [0.5, -0.3, 0.8, 0.0, 0.2, 0.0]}"
```

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions/velocities |
| `/effort_controller/commands` | `std_msgs/Float64MultiArray` | Torque commands from MPC |
| `/space_arm/target_joint_state` | `sensor_msgs/JointState` | Desired joint configuration |
| `/space_arm/ee_camera/image_raw` | `sensor_msgs/Image` | End-effector camera feed |
| `/space_arm/detected_target_pose` | `geometry_msgs/PoseStamped` | Detected target 6-DOF pose |
| `/space_arm/servo_velocity` | `geometry_msgs/TwistStamped` | Visual servoing velocity output |

## MPC Controller

The controller uses CasADi with IPOPT to solve a nonlinear optimal control problem at each timestep:

- **Dynamics**: Double-integrator per joint (appropriate for microgravity -- no gravity compensation needed)
- **Cost**: Quadratic tracking cost on position error + control effort regularization + terminal cost
- **Constraints**: Joint torque limits, velocity limits
- **Integration**: 4th-order Runge-Kutta
- **Warm starting**: Previous solution seeds the next solve

Parameters are in `space_arm_control/config/mpc_params.yaml`.

## Visual Servoing

The vision pipeline implements Image-Based Visual Servoing (IBVS):

1. **Target Detector** (`target_detector_node.py`): Detects ArUco markers or colored targets in the camera feed, estimates 6-DOF pose via PnP
2. **Visual Servoing** (`visual_servoing_node.py`): Computes camera velocity from image feature errors using the interaction matrix (image Jacobian)

## File Structure

```
.devcontainer/
├── devcontainer.json          # Codespace configuration
├── Dockerfile                 # ROS2 Humble + all dependencies
└── post-create.sh             # Auto-build on Codespace creation
src/
├── space_arm_description/
│   ├── urdf/
│   │   ├── space_arm.urdf.xacro   # Main robot model
│   │   ├── arm_macros.xacro       # Reusable link/joint macros
│   │   ├── camera.xacro           # End-effector camera
│   │   ├── gazebo.xacro           # Gazebo materials + plugins
│   │   └── materials.xacro        # Color definitions
│   ├── config/joint_limits.yaml
│   ├── launch/display.launch.py
│   └── rviz/display.rviz
├── space_arm_control/
│   ├── space_arm_control/
│   │   └── mpc_solver.py          # CasADi MPC formulation
│   ├── src/
│   │   ├── mpc_controller_node.py # ROS2 MPC node
│   │   └── joint_space_mpc.py     # Demo waypoint sequencer
│   ├── config/
│   │   ├── controllers.yaml       # ros2_control config
│   │   └── mpc_params.yaml        # MPC tuning parameters
│   └── launch/mpc_control.launch.py
├── space_arm_vision/
│   ├── src/
│   │   ├── visual_servoing_node.py
│   │   └── target_detector_node.py
│   ├── config/vision_params.yaml
│   └── launch/vision.launch.py
└── space_arm_simulation/
    ├── worlds/space_environment.world  # Microgravity Gazebo world
    ├── launch/
    │   ├── gazebo.launch.py            # Start Gazebo
    │   ├── spawn_arm.launch.py         # Spawn arm + controllers
    │   └── full_simulation.launch.py   # Everything together
    └── config/simulation_params.yaml
```

## Next Steps

1. **Inverse Kinematics**: Convert Cartesian EE targets to joint space
2. **Free-floating dynamics**: Model reaction torques on the spacecraft base
3. **MoveIt2 integration**: Motion planning with collision avoidance
4. **Learning-based control**: Train a RL policy (PPO/SAC) to augment the MPC
5. **Depth estimation**: Stereo camera or depth sensor for visual servoing
6. **Docking scenario**: Autonomous approach, alignment, and capture
7. **Multi-arm coordination**: Dual-arm manipulation
8. **Custom meshes**: Replace primitives with CAD models

## Codespaces Tips

- Use at least a **4-core** Codespace for Gazebo. 8-core recommended.
- Access Gazebo/RViz GUI through **noVNC on port 6080**.
- If VNC is slow, run `headless:=true` and inspect via topics/RViz only.
- Workspace persists across Codespace restarts. Rebuild only after changing `devcontainer.json`.

## License

MIT
