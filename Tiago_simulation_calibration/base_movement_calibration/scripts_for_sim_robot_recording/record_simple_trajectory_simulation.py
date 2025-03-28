#!/usr/bin/env python3
"""
This script controls a robot in a ROS 2 environment by publishing velocity commands to a specific topic
and recording sensor data during the robot's movement. The script takes four arguments: linear velocity in the x and y directions,
angular velocity around the z-axis, and the duration (in seconds) for which to execute the command.

Usage: python3 script.py <X> <Y> <Z> <secs>
    - X: Linear velocity in the x direction (float)
    - Y: Linear velocity in the y direction (float)
    - Z: Angular velocity around the z-axis (float)
    - secs: Duration for which to execute the movement (float)
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

def main(x, y, z, secs, folder):

    inverse_command_dif_to_forward_command = 0.0

    if secs <= inverse_command_dif_to_forward_command:
        print("Duration 'secs' should be greater than 1 for the inverse command to run.")
        inverse_duration = 0
    else:
        inverse_duration = secs - inverse_command_dif_to_forward_command

    # Set ROS domain ID
    os.environ['ROS_DOMAIN_ID'] = '68'

    # Create recordings directory if it doesn't exist
    try:
        os.makedirs(folder)
    except OSError:
        pass  # Directory already exists

    bag_filename = "x={}y={}z={}secs={}.bag".format(x, y, z, secs)
    bag_filepath = os.path.join(folder, bag_filename)

    # Remove existing bag file if it exists
    if os.path.exists(bag_filepath):
        os.remove(bag_filepath)

    # Define the ROS 2 bag record command
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

    # Start time from the clock
    start_time = clock_listener.current_time
    print("start_time (sim time): {}".format(start_time))
    print("Started publishing initial command at 100 Hz")

    rate = 0.01  # 100 Hz frequency
    try:
        # --- First command: Publish initial command for 'secs' seconds ---
        while clock_listener.current_time - start_time < secs:
            command_publisher.publish_command(x, y, z)
            # print("Published initial command")
            time.sleep(rate)
            rclpy.spin_once(clock_listener, timeout_sec=0)

        # # --- Second command: Publish inverse command for 'secs - inverse_command_dif_to_forward_command' seconds ---
        # inverse_start_time = clock_listener.current_time
        # inverse_duration = secs - inverse_command_dif_to_forward_command

        # print("Started publishing inverse command at 100 Hz")
        # while clock_listener.current_time - inverse_start_time < inverse_duration:
        #     command_publisher.publish_command(-x, -y, -z)
        #     # print("Published inverse command")
        #     time.sleep(rate)
        #     rclpy.spin_once(clock_listener, timeout_sec=0)

    finally:
        # Send stop command
        command_publisher.publish_command(0.0, 0.0, 0.0)
        print("Published stop command")

        # Allow time for the stop command to be processed
        time.sleep(0.1)

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
    if len(sys.argv) != 6:
        print("Usage: python3 script.py <X> <Y> <Z> <secs> folder")
        sys.exit(1)
    X = float(sys.argv[1])
    Y = float(sys.argv[2])
    Z = float(sys.argv[3])
    secs = float(sys.argv[4])
    folder = sys.argv[5]
    main(X, Y, Z, secs, folder)
