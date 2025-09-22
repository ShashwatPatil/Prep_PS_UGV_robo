# Prep_PS_UGV_robo

A ROS2 simulation package for the BamBot - an unmanned ground vehicle with dual manipulator arms for exploration and manipulation tasks.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble (installation instructions below)

## ROS2 Installation

If you don't have ROS2 Humble installed, follow the official installation guide:
[ROS2 Humble Installation on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## Package Dependencies

After installing ROS2 Humble, install the required packages:

### Basic ROS2 Packages
```bash
sudo apt update
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-velocity-controllers
sudo apt install ros-humble-joint-trajectory-controller
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
```
> Note: Resolve any other dependency issue you may have to run the simulation

### Additional Dependencies
```bash
sudo apt install ros-humble-xacro
sudo apt install gazebo
```

## Building the Project
> Before building download all the models/assets for the worlds form [here](https://drive.google.com/file/d/1eYnjWzUIbmfEQEDjzQg6lR0XCCsRgHs2/view?usp=sharing) and extract all of its contents into ~/.gazebo/models folder

> Also edit the bambot_v2.urdf replace all `/full_path_to_workspace/` with the actual path of your workspace

1. Clone the repository

2. Build the workspace:
```bash
cd ~/your_ws
colcon build
```

1. Source the workspace:
```bash
source install/setup.bash
```

## Running the Simulation

Launch the BamBot simulation with Gazebo:

```bash
ros2 launch gazebo_spawn_pkg spawn_robot_launch.py
```

This will:
- Start Gazebo with the default exploration world
- Spawn the BamBot robot
- Initialize all controllers (arms, wheels, head)
- Launch the robot state publisher

## Robot Control Commands/APIs

### Arm Control (Position control)

#### Right Arm Control
```bash
ros2 action send_goal /right_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['R_Rotation', 'R_Pitch', 'R_Elbow', 'R_Wrist_Pitch', 'R_Wrist_Roll', 'R_Jaw']
  points:
  - positions: [3.14, 3.14, 3.14, 3.14, 1.57, 3.14]
    time_from_start: {sec: 2}
"
```

#### Left Arm Control
```bash
ros2 action send_goal /left_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['L_Rotation', 'L_Pitch', 'L_Elbow', 'L_Wrist_Pitch', 'L_Wrist_Roll', 'L_Jaw']
  points:
  - positions: [3.14, 3.14, 3.14, 3.14, 1.57, 3.14]
    time_from_start: {sec: 2}
"
```

### Head Control
```bash
ros2 action send_goal /head_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['base_to_cam']
  points:
  - positions: [0]
    time_from_start: {sec: 1}
"
```

### Wheel Control (Individual Wheels) (velocity control)

#### Left Wheel
```bash
ros2 topic pub left_wheel_controller/commands std_msgs/msg/Float64 "data: 2.0" --once
```

#### Right Wheel
```bash
ros2 topic pub right_wheel_controller/commands std_msgs/msg/Float64 "data: -2.0" --once
```

#### Back Wheel
```bash
ros2 topic pub /back_wheel_controller/commands std_msgs/msg/Float64 "data: 1.0" --once
```

## World Files

The package includes two world files for different scenarios:

- **explore.world**: Default world for exploration tasks
- **manipulation.world**: World optimized for manipulation tasks

To change the world file, modify line 27 in `gazebo_spawn_pkg/launch/spawn_robot_launch.py`:
```python
# For exploration (default)
'explore.world'

# For manipulation tasks
'manipulation.world'
```

## Troubleshooting

### Common Issues

1. **Controller not spawning**: Ensure all ROS2 controller packages are installed
2. **Gazebo crashes**: Check that `QT_QPA_PLATFORM` is set correctly (handled in launch file)
3. **Robot not appearing**: Verify URDF file paths and mesh dependencies
