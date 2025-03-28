#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

class JointCommandPublisher(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('joint_command_publisher')
        
        # Create a publisher for the /joint_command topic
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        
        # Create a timer to allow for repeated publishing
        self.timer = self.create_timer(1.0, self.publish_joint_command)

    def publish_joint_command(self):
        # Create a JointState message
        joint_command = JointState()
        joint_command.header.stamp = self.get_clock().now().to_msg()  # Set the current time
        joint_command.name = [
            "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint", 
            "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint", "arm_right_1_joint", 
            "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint", 
            "arm_right_6_joint", "arm_right_7_joint", "gripper_left_left_finger_joint", 
            "gripper_left_right_finger_joint", "gripper_right_left_finger_joint", 
            "gripper_right_right_finger_joint", "head_1_joint", "head_2_joint", 
            "torso_lift_joint"
        ]
        joint_command.position = [
            -1.099980874501373, 1.4678273090137908, 2.713935514523769, 1.709324554831183, 
            -1.5708802004074314, 1.3897802457333837, 8.051743550977625e-05, -1.099988484910031, 
            1.467875930925463, 2.7139709201306967, 1.7095125615392166, -1.5707825220418992, 
            1.3897591943287166, -3.891224376208419e-05, 0.041745346224317324, 0.0008456588453300137, 
            0.02972811526881947, 0.04427229520093367, -0.002521365341864876, -0.9934971129967711, 
            0.1499678082727344
        ]
        
        self.pub.publish(joint_command)
        self.get_logger().info('Joint command published!')

def main(args=None):
    os.environ['ROS_DOMAIN_ID'] = '68'
    rclpy.init(args=args)
    
    node = JointCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
