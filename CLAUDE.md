# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is a ROS2 Humble workspace (`star_tracker_ws`) that integrates the SO-ARM robot with MoveIt for robotic manipulation and star tracking capabilities. The workspace combines robotic arm control, celestial tracking, and Docker containerization for cross-platform deployment.

## Core Packages

### 1. **so_100_arm**
- **Main Package**: Complete SO-100 robot configuration
- MoveIt2 integration with Gazebo simulation support
- Launch files for hardware, simulation, and visualization
- ROS2 Control integration with joint trajectory controllers
- Contains complete robot model and controller configuration

### 2. **so_arm_description**
- Robot URDF and mesh files for the SO-ARM robot
- Contains robot kinematics and visual models
- Dependencies: xacro, robot_state_publisher

### 3. **so_arm_moveit_config**
- MoveIt configuration for motion planning
- Joint limits, planning groups, and controllers
- **Key Issue**: Requires `warehouse_ros_mongo` which is unavailable on ARM64 - skip with `--skip-keys "warehouse_ros_mongo"` during rosdep install

### 4. **so_arm_100_hardware**
- ROS2 Control hardware interface for SO-ARM100
- Supports both serial communication and topic-based control
- Includes SCServo library for servo motor control
- Contains calibration scripts: `calibrate_arm.py`, `zero_pose.py`, `jog_joints.py`

### 5. **star_tracker**
- **Primary Package**: GPS-enhanced celestial tracking system
- Transforms the robot into a precision star tracker
- Features: GPS integration, IMU-based GoTo mode, multi-target support
- Comprehensive testing framework with Docker integration

## Build Commands

### Standard Build (Native)
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Install dependencies (skip problematic package on ARM64)
rosdep install --from-paths src --ignore-src -r -y --skip-keys "warehouse_ros_mongo"

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Docker Build (Recommended for ARM64/Raspberry Pi)
```bash
# Build Docker image
./docker_build.sh

# Run container
./docker_run.sh

# Or access running container
./docker_exec.sh
```

### Single Package Build
```bash
colcon build --packages-select <package_name>
```

## Testing

### Star Tracker Testing Suite
```bash
# Comprehensive validation (Docker)
./run_tests.sh

# Individual test categories
./run_tests.sh astropy      # Astronomical calculations
./run_tests.sh ros2         # ROS2 compatibility
./run_tests.sh gps          # GPS simulation

# Manual testing
python3 star_tracker/validate_tests.py
python3 star_tracker/integration_tests.py
```

### Hardware Testing
```bash
# Test servo communication
ros2 run so_arm_100_hardware test_servo

# Test ping to servos
ros2 run so_arm_100_hardware ping_test

# Calibrate arm
ros2 run so_arm_100_hardware calibrate_arm.py
```

## Launch Commands

### SO-100 Robot Control
```bash
# Launch hardware interface (physical robot)
ros2 launch so_100_arm hardware.launch.py

# Launch in Gazebo simulation
ros2 launch so_100_arm gz.launch.py dof:5

# Launch RViz visualization
ros2 launch so_100_arm rviz.launch.py

# MoveIt2 demo
ros2 launch so_100_arm moveit.launch.py
```

### MoveIt Demo (Alternative Configuration)
```bash
ros2 launch so_arm_moveit_config demo.launch.py
```

### Star Tracker (GPS-Enhanced)
```bash
# Auto GPS tracking
ros2 launch star_tracker star_tracker_gps.launch.py target_object:=moon

# GPS + IMU GoTo mode
ros2 launch star_tracker star_tracker_gps.launch.py use_imu:=true goto_mode:=true

# Manual coordinates
ros2 launch star_tracker star_tracker.launch.py location_lat:=37.7749 location_lon:=-122.4194
```

## Architecture Overview

### Control Flow
1. **High Level**: Star tracker calculates celestial positions using Astropy
2. **Motion Planning**: MoveIt plans trajectories to target coordinates
3. **Hardware Interface**: ROS2 Control translates commands to servo positions
4. **Physical Layer**: SCServo library communicates with robot hardware

### Coordinate Systems
- **Celestial**: Alt/Az coordinates (altitude/azimuth)
- **Robot**: Joint angles for 5-DOF SO-ARM configuration
- **GPS**: WGS84 coordinates for precise observer location
- **IMU**: Euler angles for orientation feedback

### Communication Protocols
- **Serial**: Direct UART communication with servo controllers
- **Topics**: ROS2 topic-based control for simulation/testing
- **GPS**: NMEA sentence parsing from Adafruit Ultimate GPS v3
- **IMU**: I2C communication with BNO055 9-DOF sensor

## Docker Architecture

### Multi-Stage Build
- **Base**: ROS2 Humble on ARM64 Ubuntu 22.04
- **Dependencies**: MoveIt, ROS2 Control, hardware interfaces
- **Workspace**: Built packages with SCServo integration
- **Runtime**: Entrypoint with proper environment sourcing

### Volume Strategy
- **Source Only**: Mount `./src` for development
- **Built Artifacts**: Keep build/install/log in container image
- **Devices**: Pass-through for USB/serial hardware access

## Hardware Integration

### SO-ARM100 Robot
- **5-DOF Configuration**: Shoulder rotation/pitch, elbow, wrist pitch/roll
- **Servo Control**: SCServo protocol via UART
- **Joint Limits**: Defined in MoveIt configuration files

### GPS Module (Adafruit Ultimate GPS v3)
- **Interface**: UART (GPIO14/15 on Raspberry Pi)
- **Protocol**: NMEA sentence parsing
- **Accuracy**: ±3m position, ±40ns time synchronization

### IMU Module (BNO055)
- **Interface**: I2C (GPIO2/3 on Raspberry Pi)
- **Features**: 9-DOF fusion, calibration states
- **Output**: Quaternions, Euler angles, linear acceleration

## Common Development Tasks

### Adding New Celestial Targets
1. Add target definition in `star_tracker/celestial_objects.py`
2. Update coordinate calculation in `star_tracker_node.py`
3. Add validation test in `test_astropy_validation.py`

### Modifying Robot Configuration
1. Update URDF in `so_arm_description/urdf/` or `so_100_arm/urdf/`
2. Regenerate MoveIt config with Setup Assistant: `ros2 launch so_100_arm setup_assistant.launch.py`
3. Adjust joint limits in `so_100_arm/config/joint_limits.yaml`
4. Update hardware interface parameters in `so_100_arm/config/ros2_controllers.yaml`

### Hardware Calibration
1. Run servo calibration: `ros2 run so_arm_100_hardware calibrate_arm.py`
2. Test servo communication: `ros2 run so_arm_100_hardware test_servo`
3. Perform star alignment: Launch with `alignment_method:=2star`
4. Save calibration: Results stored in `~/star_alignment.json`

### Robot Control Testing
```bash
# Test joint trajectory controller
ros2 action send_goal /so_100_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll],
    points: [{
      positions: [-0.5, -1.0, 0.5, 0.0, 0.0],
      time_from_start: {sec: 2}
    }]
  }
}"

# Test gripper control
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.5, max_effort: 50.0}}"
```

## Troubleshooting

### Build Issues
- **warehouse_ros_mongo**: Skip with `--skip-keys "warehouse_ros_mongo"`
- **Missing dependencies**: Run `sudo apt install ros-humble-<package>`
- **Permission errors**: Add user to dialout group: `sudo usermod -a -G dialout $USER`

### Docker Issues
- **Container exits**: Check entrypoint script and volume mounts
- **Build failures**: Use Dockerfile.fast for quicker ARM64 builds
- **Device access**: Ensure proper device mounting in docker-compose.yml

### Hardware Issues
- **GPS no fix**: Wait 30-60 seconds for cold start acquisition
- **IMU not detected**: Enable I2C with `sudo raspi-config`
- **Servo communication**: Check UART permissions and port settings

## Performance Considerations

### Tracking Accuracy
- **GPS**: ±3m position accuracy enables sub-degree pointing
- **IMU**: ±1° orientation after calibration
- **Combined**: <0.5° RMS error suitable for astrophotography

### Update Rates
- **Star Tracking**: 1-2 Hz sufficient for celestial motion
- **IMU Feedback**: 50 Hz for smooth GoTo corrections
- **GPS Updates**: 1 Hz standard rate

### Resource Usage
- **Memory**: ~200MB typical operation
- **CPU**: Minimal load on ARM64 platforms
- **Storage**: <1GB for full workspace including dependencies

## Integration Points

### Isaac Sim Integration
- Import URDF with specific damping/stiffness values
- Create ROS2 Action Graph for joint command interface
- Configure topic-based control: `/isaac_joint_states`, `/isaac_joint_command`

### External Software
- **Stellarium**: Compatible coordinate systems
- **KStars**: INDI driver potential
- **Astrophotography**: Image stacking software integration
- **Web Control**: Remote monitoring and control interfaces