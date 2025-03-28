import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import json
import sys
import subprocess
import os
import signal

class ROS2CmdVelPlayer(Node):
    def __init__(self, json_file):
        super().__init__('ros2_cmd_vel_player')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        with open(json_file, 'r') as f:
            self.cmd_vel_data = json.load(f)

        self.current_sim_time = None
        self.start_time_ros1 = self.cmd_vel_data[0]['time']
        self.current_index = 0

    def clock_callback(self, msg):
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.publish_commands()

    def publish_commands(self):
        if self.current_sim_time is None:
            return

        while self.current_index < len(self.cmd_vel_data):
            entry = self.cmd_vel_data[self.current_index]
            time_to_publish_ros1 = entry['time'] - self.start_time_ros1

            if self.current_sim_time >= time_to_publish_ros1:
                msg = Twist()
                msg.linear.x = entry['linear']['x']
                msg.linear.y = entry['linear']['y']
                msg.linear.z = entry['linear']['z']
                msg.angular.x = entry['angular']['x']
                msg.angular.y = entry['angular']['y']
                msg.angular.z = entry['angular']['z']

                self.cmd_vel_pub.publish(msg)
                self.get_logger().info(f'Published cmd_vel: {msg}')

                self.current_index += 1
            else:
                break

        if self.current_index >= len(self.cmd_vel_data):
            self.get_logger().info("All commands have been published. Shutting down...")
            rclpy.shutdown()

# Purpose:
# Allows playing a recorded sequence of cmd_vel commands from .json file
# and saves the resulting simulated odometry data into a ros2 bag file
def main(args=None):
    os.environ['ROS_DOMAIN_ID'] = '68'

    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: python3 play_ros2_cmd_vel.py <json_file> <resulting_odom_bag_file>")
        return

    json_file = sys.argv[1]
    bag_file = sys.argv[2]

    record_command = ['ros2', 'bag', 'record', '-o', bag_file, '/joint_states', '/clock', '/odom', "/cmd_vel"]
    process = subprocess.Popen(record_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        player = ROS2CmdVelPlayer(json_file)
        rclpy.spin(player)
    finally:
        # Ensure the recording process is terminated
        process.send_signal(signal.SIGINT)
        stdout, stderr = process.communicate()
        print(stdout.decode())
        print(stderr.decode())
        process.wait()
        print(f"Finished recording to {bag_file}")


if __name__ == '__main__':
    main()
