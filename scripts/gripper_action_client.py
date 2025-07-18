#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dh_ag95_gripper.action import GripperCommand
import time

class GripperActionClient(Node):
    def __init__(self):
        super().__init__('gripper_action_client')
        
        # 创建Action客户端
        self._action_client = ActionClient(self, GripperCommand, 'gripper_command')
        
        self.get_logger().info('夹爪Action客户端已启动')
    
    def send_goal(self, position, max_effort=50.0, timeout=10.0):
        """
        发送夹爪控制目标
        
        Args:
            position (float): 目标位置 (0.0=完全打开, 1.0=完全关闭)
            max_effort (float): 最大力度
            timeout (float): 超时时间（秒）
            
        Returns:
            bool: 是否成功到达目标
        """
        # 等待Action服务器
        if not self._action_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('Action服务器不可用')
            return False
        
        # 创建目标
        goal_msg = GripperCommand.Goal()
        goal_msg.position = position
        goal_msg.max_effort = max_effort
        
        self.get_logger().info(f'发送夹爪控制目标: 位置={position:.3f}, 最大力={max_effort:.3f}')
        
        # 发送目标并等待结果
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # 等待目标被接受
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return False
        
        self.get_logger().info('目标被接受，开始执行')
        
        # 等待结果
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result()
        
        # 处理结果
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('目标成功完成！')
            success = result.result.reached_goal
        elif result.status == 5:  # ABORTED
            self.get_logger().error('目标被中止')
            success = False
        elif result.status == 6:  # CANCELED
            self.get_logger().warn('目标被取消')
            success = False
        else:
            self.get_logger().error(f'未知结果状态: {result.status}')
            success = False
        
        # 显示最终结果
        self.get_logger().info(
            f'最终结果: 位置={result.result.position:.3f}, '
            f'力={result.result.effort:.3f}, '
            f'卡住={"是" if result.result.stalled else "否"}, '
            f'到达目标={"是" if result.result.reached_goal else "否"}'
        )
        
        return success
    
    def feedback_callback(self, feedback_msg):
        """反馈回调函数"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'反馈: 位置={feedback.position:.3f}, '
            f'力={feedback.effort:.3f}, '
            f'卡住={"是" if feedback.stalled else "否"}'
        )
    
    def open_gripper(self, max_effort=30.0):
        """打开夹爪"""
        return self.send_goal(0.0, max_effort)
    
    def close_gripper(self, max_effort=50.0):
        """关闭夹爪"""
        return self.send_goal(1.0, max_effort)
    
    def set_gripper_position(self, position, max_effort=40.0):
        """设置夹爪位置"""
        return self.send_goal(position, max_effort)


def main(args=None):
    rclpy.init(args=args)
    
    client = GripperActionClient()
    
    try:
        # 示例：控制夹爪
        print("=== 夹爪控制示例 ===")
        
        # 1. 打开夹爪
        print("1. 打开夹爪")
        success = client.open_gripper()
        if success:
            print("✓ 夹爪成功打开")
        else:
            print("✗ 夹爪打开失败")
        
        time.sleep(2)
        
        # 2. 关闭夹爪
        print("2. 关闭夹爪")
        success = client.close_gripper()
        if success:
            print("✓ 夹爪成功关闭")
        else:
            print("✗ 夹爪关闭失败")
        
        time.sleep(2)
        
        # 3. 半开状态
        print("3. 设置半开状态")
        success = client.set_gripper_position(0.5)
        if success:
            print("✓ 夹爪成功设置为半开")
        else:
            print("✗ 夹爪设置失败")
        
    except KeyboardInterrupt:
        print("程序被中断")
    
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 