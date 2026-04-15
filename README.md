# UR5e Robot Control with Robotiq Gripper

A comprehensive Python control system for Universal Robots UR5e collaborative robot with Robotiq HAND-E gripper integration. This project provides both command-line and GUI interfaces for robot control, including joint/TCP movement, waypoint navigation, and gripper operations.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for full details.

## Features

- **RTDE Communication**: Direct real-time control via Universal Robots RTDE (Real-Time Data Exchange) interface
- **Gripper Control**: Full Robotiq gripper support:
  - Activate/deactivate gripper
  - Open/close operations
  - Position control with force and speed settings
- **Motion Control**:
  - Joint space movement (moveJ)
  - Linear space movement (moveL)
  - Relative movements in world and tool frames
  - Incremental joint adjustments
- **GUI Interface**: Tkinter-based GUI for intuitive robot control
  - Real-time joint position display
  - Translation and rotation controls
  - Waypoint management
  - Gripper control panel
- **Automatic Reconnection**: Background monitoring and automatic reconnection on connection loss
- **Home Position & Waypoints**: Predefined positions for quick navigation

The communication with the robot uses the `ur_rtde` library which is UR’s official RTDE Python client: <https://sdurobotics.gitlab.io/ur_rtde/>

Functionality has been wrapped into a control script which includes transforms for relative tool movements and gripper commands. More information about the transforms and gripper control can be found here:
- **Robotiq Gripper Control** (through UR Cap port): <https://sdurobotics.gitlab.io/ur_rtde/guides/guides.html#use-with-robotiq-gripper>
- **Relative Tool Movements**: <https://www.robotexchange.io/t/how-to-control-a-ur-robot-from-python-using-rtde/3271>

## Project Structure

```
UR5e_with_robotiq_gripper_control/
├── robotiq_gripper.py      # Robotiq gripper communication module
├── ur_control.py           # Main robot control interface (RTDE)
├── ur_gui.py               # GUI application for robot control
├── requirements.txt        # Python dependencies
└── README.md               # This file
```

## System Requirements

- **Python**: 3.7+
- **Operating System**: Linux, macOS, or Windows
- **Network**: Direct network connection to UR5e robot (port 30001 for RTDE, port 63352 for gripper)
- **UR Robot Software**: 3.13+

## Installation

1. **Clone/Download Project**:
   ```bash
   git clone <repository-url>
   cd UR5e_with_robotiq_gripper_control
   ```

2. **Install Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Verify Python & Tkinter**:
   - Python 3.7+: `python --version`
   - Tkinter: `python -c "import tkinter; print('Tkinter OK')"` 
   - On Linux, you may need to install tkinter: `sudo apt-get install python3-tk`

## Configuration

### Robot IP Address

Update the robot IP address in the relevant files:

- **ur_gui.py** (line ~29):
  ```python
  self.robot = URControl("192.168.1.3")  # <-- change IP
  ```

### Home Position & Waypoints

Modify the home position and waypoint coordinates in [ur_control.py](ur_control.py#L25-L33):

```python
self.home_position = (-1.571, -1.396, -2.531, 0.785, 1.571, 4.712)

self.waypoints = {
    "waypoint_1": (-1.396, -2.217, -0.785, -1.658, 1.571, 3.316),
    "waypoint_2": (-3.153, -3.123, -1.074, -1.990, -1.571, 4.719),
    "waypoint_3": (-3.153, -2.628, -1.340, 0.801, 1.686, 4.633),
}
```

Values are in radians for joint angles and meters/radians for TCP poses.

## Usage

### GUI Application (Recommended)

Launch the graphical interface:
```bash
python ur_gui.py
```

**GUI Features**:
- **Translation Panel**: X, Y, Z movement controls with step size adjustment
- **Rotation Panel**: Rx, Ry, Rz rotation controls
- **Joint Control**: Individual joint adjustment with position feedback
- **Robot Status**: Home position, joint/TCP position readout
- **Waypoints**: Quick navigation to predefined positions
- **Gripper Control**: Activate, open, and close gripper
- **Frame Selection**: Toggle between world and tool frame for movements
- **Reconnect**: Manual reconnection button for dropped connections

### Programmatic Control

Import and use the control interface in your scripts:

```python
from ur_control import URControl, GripperNotActivatedError

# Initialize
robot = URControl("192.168.1.3")
robot.connect()
robot.connect_gripper()

# Move to home position
robot.move_home()

# Get current positions
joint_pos = robot.get_joint_pos()
tcp_pos = robot.get_tcp_pos()

# Move to joint position
robot.move_j([-1.571, -1.396, -2.531, 0.785, 1.571, 4.712])

# Move linearly
robot.move_l([x, y, z, rx, ry, rz])

# Relative movements
robot.relative_world_move([0.1, 0, 0, 0, 0, 0])  # Move 0.1m in X
robot.relative_tool_move([0, 0.05, 0, 0, 0, 0])  # Move 0.05m in tool Z

# Gripper operations
robot.gripper_activate()
robot.gripper_open()
robot.gripper_close()
robot.gripper_move(position=128, speed=255, force=255)

# Cleanup
robot.stop_robot_control()
```

## Module Documentation

### robotiq_gripper.py

Socket-based communication with Robotiq HAND-E gripper via port 63352.

**Key Methods**:
- `connect(hostname, port, socket_timeout)`: Establish gripper connection
- `activate(auto_calibrate)`: Activate and calibrate gripper
- `move_and_wait_for_pos(position, speed, force)`: Move and wait for completion
- `is_open()`, `is_closed()`: Check gripper state
- `get_current_position()`: Get actual gripper position

**Attributes**:
- Position range: 0 (fully closed) to 255 (fully open)
- Speed range: 0–255
- Force range: 0–255

### ur_control.py

Main RTDE-based robot control interface.

**Key Methods**:
- `connect()`: Connect to robot (with retry logic)
- `connect_gripper()`: Connect to gripper
- `move_j(joint_positions, speed, acceleration)`: Joint space move
- `move_l(tcp_pose, speed, acceleration)`: Cartesian space move
- `move_add_j(joint_deltas)`: Incremental joint move
- `relative_world_move(pose_delta, speed, acceleration)`: World frame relative move
- `relative_tool_move(pose_delta, speed, acceleration)`: Tool frame relative move
- `move_to_waypoint(waypoint_name)`: Move to predefined waypoint
- `gripper_activate()`, `gripper_open()`, `gripper_close()`, `gripper_move()`: Gripper control

**Features**:
- Automatic reconnection on connection loss (background monitor thread)
- Thread-safe socket operations
- Comprehensive error handling and logging

### ur_gui.py

Tkinter-based graphical user interface.

**Main Features**:
- Real-time joint position display
- Press-and-hold continuous motion
- Adjustable step sizes for translation and rotation
- World/tool frame toggle
- Visual feedback for all operations

## Common Use Cases

### Pick and Place Operation

```python
robot = URControl("192.168.1.3")
robot.connect()
robot.connect_gripper()

# Move to pick location
robot.move_to_waypoint("waypoint_1")
robot.gripper_activate()
robot.gripper_close()

# Move to place location
robot.move_to_waypoint("waypoint_2")
robot.gripper_open()

# Return home
robot.move_home()
robot.stop_robot_control()
```

### Incremental Adjustments

```python
# Fine-tune position using small increments
robot.relative_world_move([0.01, 0, 0, 0, 0, 0])  # Move 1cm in X
robot.move_add_j([0, 0.05, 0, 0, 0, 0])  # Adjust shoulder joint
```

## Troubleshooting

### Connection Issues
- Verify robot IP address is correct
- Ensure network connectivity: `ping 192.168.1.3`
- Check that robot is powered on and in remote control mode
- Verify firewall doesn't block ports 30001 (RTDE) and 63352 (gripper)

### Gripper Not Responding
- Check gripper is powered on
- Verify IP and port (default: 63352)
- Call `gripper_activate()` before gripper operations
- Check gripper cable connection

### GUI Freezing
- If robot loses connection, the background monitor will attempt reconnection
- Use the "Reconnect" button to manually reconnect
- Check logs for detailed error messages

### Tkinter Not Available
Linux users may need to install tkinter:
```bash
sudo apt-get install python3-tk
```

## Logging

The application logs detailed information to console. Adjust logging level in [ur_control.py](ur_control.py#L7-L12):

```python
logging.basicConfig(
    level=logging.DEBUG,  # Change to INFO for less verbose output
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
```

## Safety Considerations

- Always ensure the workspace is clear before running automated movements
- Test movements with reduced speed/force settings first
- Use the emergency stop on the robot teach pendant if needed
- Never leave the robot unattended during operation
- Verify waypoint positions are safe before use

## Support & Contributions

For issues, questions, or contributions, please [add your contact/repository information].

## References

- [Universal Robots RTDE Documentation](https://www.universal-robots.com/articles/ur/software-architecture/)
- [Robotiq Gripper Documentation](https://assets.robotiq.com/website-assets/Support_Documents/Documentation/Robotiq-Hand-E_User_Guide.pdf)
- [ur-rtde Python Package](https://github.com/UniversalRobots/RTDE_Python_Client_Library)
