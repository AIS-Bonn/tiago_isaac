#!/usr/bin/env python3
"""
This script controls a robot in a ROS 2 environment by publishing velocity commands to a specific topic
and recording sensor data during the robot's movement. The script takes two arguments: speed (for linear velocities)
and the duration (in seconds) for which to execute each movement (forward, left, backward, right).

Usage: python3 script.py <SPEED> <SECS> <folder>
    - SPEED: Linear velocity magnitude for all movements (float)
    - SECS: Duration for which to execute each movement (float)
    - folder: Folder where bag files will be stored (string)
"""

import os
import sys
import time
import rclpy
from rclpy.node import Node
import subprocess
import signal
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

class ClockListener(Node):
    def __init__(self):
        super().__init__('clock_listener')
        self.current_time = None
        self.subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
    
    def clock_callback(self, msg):
        self.current_time = msg.clock.sec + msg.clock.nanosec / 1e9

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_command(self, linear_x, linear_y, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.publisher_.publish(msg)

def execute_square_trajectory(speed, secs, folder):

    # Set ROS domain ID
    os.environ['ROS_DOMAIN_ID'] = '68'

    # Create recordings directory if it doesn't exist
    try:
        os.makedirs(folder)
    except OSError:
        pass  # Directory already exists

    # Define the ROS 2 bag record command
    bag_filename = "square_trajectory_speed={}secs={}.bag".format(speed, secs)
    bag_filepath = os.path.join(folder, bag_filename)
    if os.path.exists(bag_filepath):
        os.remove(bag_filepath)

    record_cmd = [
        'ros2', 'bag', 'record', '-o', bag_filepath, '/joint_states', '/clock', '/odom', '/cmd_vel'
    ]

    # Start ROS 2 node for clock listener and command publisher
    rclpy.init()
    clock_listener = ClockListener()
    command_publisher = CommandPublisher()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(clock_listener)
    executor.add_node(command_publisher)

    # Start recording process
    print("Starting recording process")
    record_process = subprocess.Popen(
        record_cmd,
        stdout=open(os.devnull, 'wb'),
        stderr=open(os.devnull, 'wb'),
        preexec_fn=os.setsid
    )

    # Wait for some time to allow clock to sync
    print("Waiting for clock sync...")
    while clock_listener.current_time is None:
        rclpy.spin_once(clock_listener)

    rate = 0.01  # 100 Hz frequency

    def publish_for_duration(x, y, z, duration):
        start_time = clock_listener.current_time
        while clock_listener.current_time - start_time < duration:
            command_publisher.publish_command(x, y, z)
            time.sleep(rate)
            rclpy.spin_once(clock_listener, timeout_sec=0)

    try:
        # --- First leg: Move forward ---
        print("Moving forward")
        publish_for_duration(speed, 0.0, 0.0, secs)

        # --- Second leg: Move left ---
        print("Moving left")
        publish_for_duration(0.0, -speed, 0.0, secs)

        # --- Third leg: Move backward ---
        print("Moving backward")
        publish_for_duration(-speed, 0.0, 0.0, secs)

        # --- Fourth leg: Move right ---
        print("Moving right")
        publish_for_duration(0.0, speed, 0.0, secs)

    finally:
        # Send stop command
        publish_for_duration(0.0, 0.0, 0.0, 2)
        print("Published stop command")

        # Stop the executor
        executor.shutdown()
        print("Executor shut down")

        # Destroy nodes
        command_publisher.destroy_node()
        clock_listener.destroy_node()
        print("Nodes destroyed")

        # Stop the recording process
        os.killpg(os.getpgid(record_process.pid), signal.SIGINT)
        record_process.wait()
        print("Stopped recording process")

        # Shutdown ROS 2
        rclpy.shutdown()
        print("ROS 2 shutdown")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python3 script.py <SPEED> <SECS> <folder>")
        sys.exit(1)
    SPEED = float(sys.argv[1])
    SECS = float(sys.argv[2])
    folder = sys.argv[3]
    execute_square_trajectory(SPEED, SECS, folder)
