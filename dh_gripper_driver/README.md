# DH Gripper Driver Package

This package contains the ROS2 driver for the DH AG95 gripper with serial communication and control interfaces.

## Package Contents

- **Driver node**: Main gripper driver with serial communication
- **Message definitions**: Custom messages for gripper control and state
- **Launch files**: For starting the driver
- **Scripts**: Utility scripts for testing and configuration

## Usage

### Start the gripper driver

```bash
# Start driver with default parameters
ros2 launch dh_gripper_driver dh_ag95_gripper.launch.py

# Start with custom device port
ros2 launch dh_gripper_driver dh_ag95_gripper.launch.py device_port:=/dev/ttyUSB1

# Start with custom baudrate
ros2 launch dh_gripper_driver dh_ag95_gripper.launch.py baudrate:=9600
```

## Topics

### Published Topics

- `/gripper/joint_states` (sensor_msgs/JointState): Joint state information
- `/gripper/state` (dh_gripper_driver/GripperState): Gripper status information

### Subscribed Topics

- `/gripper/ctrl` (dh_gripper_driver/GripperCtrl): Gripper control commands

## Messages

### GripperCtrl.msg
```
bool initialize
float64 position
float64 effort
bool stop
```

### GripperState.msg
```
bool initialized
float64 position
float64 effort
bool moving
bool error
string error_message
```

## Parameters

- `device_port` (string, default: "/dev/ttyUSB0"): Serial device port
- `baudrate` (int, default: 115200): Serial communication baudrate
- `gripper_id` (int, default: 1): Gripper ID for communication
- `max_position` (double, default: 100.0): Maximum gripper position in mm
- `max_force` (double, default: 100.0): Maximum gripper force in N

## Dependencies

- `rclcpp`
- `std_msgs`
- `sensor_msgs`
- `serial`

## Files Structure

```
dh_gripper_driver/
├── launch/
│   └── dh_ag95_gripper.launch.py
├── msg/
│   ├── GripperCtrl.msg
│   └── GripperState.msg
├── scripts/
│   └── (utility scripts)
├── src/
│   └── dh_ag95_driver.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
``` 