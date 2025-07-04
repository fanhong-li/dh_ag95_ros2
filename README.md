# DH AG95 Gripper ROS2 Driver

A ROS2 driver package for the DH AG95 electric gripper, providing seamless integration with ROS2 systems.

## Overview

The DH AG95 is a high-precision electric gripper designed for robotic applications. This package provides a complete ROS2 interface for controlling the gripper, including position control, force control, and status monitoring.

## Features

- **Position Control**: Precise gripper positioning (0-100mm range)
- **Force Control**: Adjustable gripping force (10-100N)
- **Speed Control**: Variable opening/closing speed (1-100%)
- **Status Monitoring**: Real-time gripper state feedback
- **Auto-initialization**: Automatic gripper calibration on startup
- **Modbus RTU Communication**: Reliable serial communication protocol

## Hardware Requirements

- DH AG95 Electric Gripper
- USB-to-RS485 converter or direct serial connection
- ROS2 Humble (tested) or newer

## Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/your-username/dh_ag95_gripper.git
```

### 2. Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Package

```bash
colcon build --packages-select dh_ag95_gripper
source install/setup.bash
```

### 4. Setup Serial Permissions

Add your user to the dialout group for serial port access:

```bash
sudo usermod -a -G dialout $USER
```

Or temporarily set permissions:

```bash
sudo chmod 666 /dev/ttyUSB0  # Replace with your device
```

## Usage

### 1. Start the Driver

```bash
ros2 run dh_ag95_gripper dh_ag95_driver
```

### 2. Control the Gripper

#### Initialize the Gripper
```bash
ros2 topic pub --once /gripper/ctrl dh_ag95_gripper/msg/GripperCtrl \
  "initialize: true
   position: 0.0
   force: 50.0
   speed: 50.0"
```

#### Set Position
```bash
ros2 topic pub --once /gripper/ctrl dh_ag95_gripper/msg/GripperCtrl \
  "initialize: false
   position: 30.0
   force: 40.0
   speed: 60.0"
```

#### Close Gripper
```bash
ros2 topic pub --once /gripper/ctrl dh_ag95_gripper/msg/GripperCtrl \
  "initialize: false
   position: 0.0
   force: 50.0
   speed: 50.0"
```

#### Open Gripper
```bash
ros2 topic pub --once /gripper/ctrl dh_ag95_gripper/msg/GripperCtrl \
  "initialize: false
   position: 100.0
   force: 30.0
   speed: 70.0"
```

### 3. Monitor Gripper Status

```bash
ros2 topic echo /gripper/state
```

## API Reference

### Topics

#### Published Topics

- `/gripper/state` ([dh_ag95_gripper/msg/GripperState](msg/GripperState.msg))
  - Current gripper status including position, force, and initialization state

#### Subscribed Topics

- `/gripper/ctrl` ([dh_ag95_gripper/msg/GripperCtrl](msg/GripperCtrl.msg))
  - Gripper control commands

### Messages

#### GripperCtrl.msg
```
bool initialize     # Initialize gripper (true/false)
float64 position    # Target position in mm (0.0-100.0)
float64 force       # Gripping force in N (10.0-100.0)
float64 speed       # Movement speed in % (1.0-100.0)
```

#### GripperState.msg
```
bool is_initialized    # Gripper initialization status
float64 position      # Current position in mm
float64 force         # Current force in N
bool is_gripping      # Object gripping status
```

### Parameters

- `device_port` (string, default: "/dev/ttyUSB0"): Serial device path
- `baud_rate` (int, default: 115200): Serial communication baud rate
- `gripper_id` (int, default: 1): Modbus device ID
- `force_initialization` (bool, default: false): Force initialization on startup

## Troubleshooting

### Common Issues

1. **Permission Denied Error**
   ```
   [ERROR] Failed to open serial port: /dev/ttyUSB0
   ```
   Solution: Add user to dialout group or set permissions:
   ```bash
   sudo usermod -a -G dialout $USER
   # or
   sudo chmod 666 /dev/ttyUSB0
   ```

2. **Device Not Found**
   ```
   [ERROR] Serial device not found
   ```
   Solution: Check device connection and path:
   ```bash
   ls /dev/ttyUSB*
   dmesg | grep tty
   ```

3. **Communication Timeout**
   ```
   [WARN] Register write failed
   ```
   Solution: Check wiring, baud rate, and device ID settings.

### Debug Mode

To enable verbose logging:
```bash
ros2 run dh_ag95_gripper dh_ag95_driver --ros-args --log-level debug
```

## Technical Specifications

- **Communication Protocol**: Modbus RTU
- **Baud Rate**: 115200 (default)
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Position Range**: 0-100mm
- **Force Range**: 10-100N
- **Speed Range**: 1-100%

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

For issues and questions:
- Create an issue on GitHub
- Check the troubleshooting section
- Review the DH AG95 manual

## Changelog

### v1.0.0
- Initial release
- Basic position, force, and speed control
- Auto-initialization feature
- ROS2 Humble support
- Modbus RTU communication 