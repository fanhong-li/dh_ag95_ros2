# DH AG95 ROS2 驱动与描述包

本仓库为 DH Robotics AG95 电动夹爪的 ROS2 驱动与描述包，支持真实夹爪控制与 RViz 三维可视化。

---

## 目录结构

```
dh_ag95_description/   # 夹爪的URDF、网格、RViz配置等（来自 ian-chuang/dh_ag95_gripper_ros2）
dh_gripper_driver/     # 夹爪的C++驱动节点、消息定义等（改编自 DH-Robotics/dh_gripper_ros 的ROS1驱动）
```

---

## 代码来源

- **description** 目录（URDF、meshes等）  
  来源：[ian-chuang/dh_ag95_gripper_ros2](https://github.com/ian-chuang/dh_ag95_gripper_ros2)

- **driver** 目录（C++串口驱动、消息定义等）  
  来源：[DH-Robotics/dh_gripper_ros](https://github.com/DH-Robotics/dh_gripper_ros)  
  已适配为 ROS2 版本，支持标准的 ROS2 消息和 launch 方式。

---

## 主要特性

- 只需发布主关节（如 `left_outer_knuckle_joint`）的角度，其他关节由 URDF mimic 自动推导
- 支持夹爪位置、力、速度控制
- 支持 RViz 实时三维显示
- 兼容 ROS2 Humble 及以上

---

## 快速使用

### 1. 构建

```bash
colcon build
source install/setup.bash
```

### 2. 启动夹爪驱动与可视化

```bash
ros2 launch dh_gripper_driver dh_ag95_gripper.launch.py
```

### 3. 控制夹爪

```bash
ros2 topic pub /gripper/ctrl dh_gripper_driver/msg/GripperCtrl "{initialize: false, position: 50.0, force: 50.0, speed: 20.0}"
```

### 4. 查看关节状态

```bash
ros2 topic echo /gripper/joint_states
```

---

## 说明

- **description** 目录下的 URDF 采用 mimic 机制，只需发布主关节即可，robot_state_publisher 会自动推导所有关节的 TF。
- **driver** 目录下的 C++ 节点通过串口与夹爪通信，发布主关节角度到 `/gripper/joint_states`，并订阅 `/gripper/ctrl` 控制夹爪。

---

## 参考项目

- [ian-chuang/dh_ag95_gripper_ros2](https://github.com/ian-chuang/dh_ag95_gripper_ros2)  
  —— 夹爪的 ROS2 机械描述与可视化配置

- [DH-Robotics/dh_gripper_ros](https://github.com/DH-Robotics/dh_gripper_ros)  
  —— 官方 ROS1 驱动，已适配为 ROS2

---

## License

MIT License

---

如需详细说明或遇到问题，欢迎提 issue！ 