# DH AG95 夹爪 ROS2 Action 接口使用说明

## 概述

本包提供了基于ROS2 Action的夹爪控制接口，适合在机械臂系统中集成使用。

## Action接口定义

### GripperCommand Action

```
# Goal: 目标位置和力度
float64 position    # 目标位置 (0.0 = 完全打开, 1.0 = 完全关闭)
float64 max_effort  # 最大力度 (可选，默认50.0)
---
# Result: 最终结果
float64 position    # 最终位置
float64 effort      # 最终力度
bool stalled        # 是否因力过大而停止
bool reached_goal   # 是否到达目标位置
---
# Feedback: 实时反馈
float64 position    # 当前位置
float64 effort      # 当前力度
bool stalled        # 是否卡住
```

## 启动服务

### 1. 启动Action服务器和驱动

```bash
# 启动夹爪驱动和Action服务器
ros2 launch dh_ag95_gripper gripper_action.launch.py

# 或者同时启动显示界面
ros2 launch dh_ag95_gripper gripper_action.launch.py start_display:=true
```

### 2. 仅启动Action服务器（不启动硬件驱动）

```bash
ros2 launch dh_ag95_gripper gripper_action.launch.py start_driver:=false
```

## 使用方法

### 1. 命令行测试

```bash
# 发送Action目标
ros2 action send_goal /gripper_command dh_ag95_gripper/action/GripperCommand "{position: 0.0, max_effort: 30.0}"

# 查看Action服务器状态
ros2 action list
ros2 action info /gripper_command
```

### 2. Python客户端

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dh_ag95_gripper.scripts.gripper_action_client import GripperActionClient

def main():
    rclpy.init()
    
    client = GripperActionClient()
    
    # 打开夹爪
    success = client.open_gripper(max_effort=30.0)
    print(f"打开夹爪: {'成功' if success else '失败'}")
    
    # 关闭夹爪
    success = client.close_gripper(max_effort=50.0)
    print(f"关闭夹爪: {'成功' if success else '失败'}")
    
    # 设置特定位置
    success = client.set_gripper_position(0.5, max_effort=40.0)
    print(f"设置位置: {'成功' if success else '失败'}")
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. C++客户端

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <dh_ag95_gripper/action/gripper_command.hpp>

class MyRobotNode : public rclcpp::Node
{
public:
    MyRobotNode() : Node("my_robot_node")
    {
        // 创建Action客户端
        gripper_client_ = rclcpp_action::create_client<dh_ag95_gripper::action::GripperCommand>(
            this, "gripper_command");
    }
    
    void openGripper()
    {
        auto goal = dh_ag95_gripper::action::GripperCommand::Goal();
        goal.position = 0.0;  // 打开
        goal.max_effort = 30.0;
        
        gripper_client_->async_send_goal(goal);
    }
    
    void closeGripper()
    {
        auto goal = dh_ag95_gripper::action::GripperCommand::Goal();
        goal.position = 1.0;  // 关闭
        goal.max_effort = 50.0;
        
        gripper_client_->async_send_goal(goal);
    }

private:
    rclcpp_action::Client<dh_ag95_gripper::action::GripperCommand>::SharedPtr gripper_client_;
};
```

## 与机械臂集成

### 1. 在机械臂节点中集成

```python
# 在机械臂控制节点中
from dh_ag95_gripper.scripts.gripper_action_client import GripperActionClient

class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')
        
        # 创建夹爪客户端
        self.gripper_client = GripperActionClient()
        
    def pick_object(self):
        # 1. 移动机械臂到抓取位置
        self.move_arm_to_pick_position()
        
        # 2. 打开夹爪
        success = self.gripper_client.open_gripper()
        if not success:
            self.get_logger().error("夹爪打开失败")
            return False
            
        # 3. 下降接近物体
        self.move_arm_down()
        
        # 4. 关闭夹爪抓取
        success = self.gripper_client.close_gripper(max_effort=40.0)
        if not success:
            self.get_logger().error("夹爪抓取失败")
            return False
            
        # 5. 提升物体
        self.move_arm_up()
        
        return True
```

### 2. 错误处理

```python
def safe_gripper_operation(self, position, max_effort=50.0):
    """安全的夹爪操作，包含错误处理"""
    try:
        success = self.gripper_client.send_goal(position, max_effort, timeout=5.0)
        if not success:
            self.get_logger().warn(f"夹爪操作失败: 位置={position}")
            return False
        return True
    except Exception as e:
        self.get_logger().error(f"夹爪操作异常: {e}")
        return False
```

## 故障排除

### 1. Action服务器不可用

```bash
# 检查Action服务器状态
ros2 action list
ros2 node list | grep gripper

# 重启Action服务器
ros2 launch dh_ag95_gripper gripper_action.launch.py
```

### 2. 夹爪硬件连接问题

```bash
# 检查串口设备
ls -la /dev/ttyUSB*

# 检查驱动日志
ros2 topic echo /rosout
```

### 3. 位置不准确

```bash
# 监控关节状态
ros2 topic echo /joint_states

# 检查Action反馈
ros2 action send_goal /gripper_command dh_ag95_gripper/action/GripperCommand "{position: 0.5, max_effort: 30.0}" --feedback
```

## 优势

1. **异步操作**：不阻塞主程序，适合机械臂控制
2. **实时反馈**：提供位置、力度等实时状态
3. **错误处理**：支持超时、力度过大等异常情况
4. **可取消**：支持中途取消操作
5. **标准接口**：符合ROS2 Action规范，易于集成

## 注意事项

1. 位置范围：0.0（完全打开）到 1.0（完全关闭）
2. 力度单位：根据实际硬件调整
3. 超时设置：建议设置合理的超时时间
4. 错误处理：务必检查操作结果并处理异常情况 