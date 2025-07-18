#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
import time


class GripperMoveItTest(Node):
    def __init__(self):
        super().__init__('gripper_moveit_test')
        
        # Create publishers and subscribers
        self.trajectory_pub = self.create_publisher(
            DisplayTrajectory, 
            '/display_planned_path', 
            10
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Create service clients
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        self.get_logger().info('Gripper MoveIt Test Node initialized')
        
    def test_joint_states(self):
        """Test publishing joint states"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'gripper_finger1_joint',
            'gripper_finger1_finger_joint', 
            'gripper_finger2_finger_joint',
            'gripper_finger1_inner_knuckle_joint',
            'gripper_finger1_finger_tip_joint',
            'gripper_finger2_joint',
            'gripper_finger2_inner_knuckle_joint',
            'gripper_finger2_finger_tip_joint'
        ]
        
        # Open position
        joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_state.velocity = [0.0] * len(joint_state.name)
        joint_state.effort = [0.0] * len(joint_state.name)
        
        self.get_logger().info('Publishing open gripper position')
        self.joint_state_pub.publish(joint_state)
        
        time.sleep(2.0)
        
        # Closed position
        joint_state.position = [0.5, -0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5]
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info('Publishing closed gripper position')
        self.joint_state_pub.publish(joint_state)
        
    def test_ik_service(self):
        """Test inverse kinematics service"""
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('IK service not available')
            return False
            
        # Create IK request
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'gripper'
        request.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.robot_state.joint_state.name = [
            'gripper_finger1_joint',
            'gripper_finger1_finger_joint', 
            'gripper_finger2_finger_joint'
        ]
        request.ik_request.robot_state.joint_state.position = [0.0, 0.0, 0.0]
        
        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'gripper_base_link'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = 0.0
        pose_stamped.pose.position.y = 0.0
        pose_stamped.pose.position.z = 0.1
        pose_stamped.pose.orientation.w = 1.0
        
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.timeout.sec = 5
        
        try:
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response.error_code.val == response.error_code.SUCCESS:
                self.get_logger().info('IK service test successful')
                return True
            else:
                self.get_logger().warn(f'IK service returned error: {response.error_code.val}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'IK service call failed: {e}')
            return False
            
    def run_tests(self):
        """Run all tests"""
        self.get_logger().info('Starting gripper MoveIt tests...')
        
        # Test 1: Joint states
        self.test_joint_states()
        
        # Test 2: IK service
        self.test_ik_service()
        
        self.get_logger().info('Tests completed')


def main(args=None):
    rclpy.init(args=args)
    
    test_node = GripperMoveItTest()
    
    try:
        test_node.run_tests()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 