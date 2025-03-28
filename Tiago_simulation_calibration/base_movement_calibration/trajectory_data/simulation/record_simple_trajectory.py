import os
import subprocess
import signal
import sys
import time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
import shutil

class ClockListener(Node):
    def __init__(self):
        super().__init__('clock_listener')
        self.current_time = 0.0
        self.subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = None  # Timer for publishing commands
        self.command_msg = None  # The Twist message to publish
        self.subscription  # prevent unused variable warning

    def clock_callback(self, msg):
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def publish_stop_cmd(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)
        self.get_logger().info('Published stop command')

    def start_publishing_command(self, x, y, z):
        # Create the command message
        self.command_msg = Twist()
        self.command_msg.linear.x = x
        self.command_msg.linear.y = y
        self.command_msg.linear.z = 0.0
        self.command_msg.angular.x = 0.0
        self.command_msg.angular.y = 0.0
        self.command_msg.angular.z = z

        # Create a timer to publish at 100 Hz
        self.timer = self.create_timer(0.01, self.publish_command_callback)
        self.get_logger().info('Started publishing command at 100 Hz')

    def publish_command_callback(self):
        self.publisher.publish(self.command_msg)

    def stop_publishing_command(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            self.get_logger().info('Stopped publishing command')

def main(x, y, z, secs):
    # Set ROS_DOMAIN_ID environment variable
    os.environ['ROS_DOMAIN_ID'] = '68'

    os.makedirs("recordings/360/bags/", exist_ok=True)

    bag_filename = f"x={x}y={y}z={z}secs={secs}"
    bag_filepath = os.path.join("recordings/360/bags", bag_filename)

    # Check if the directory for the bag file already exists
    if os.path.exists(bag_filepath):
        shutil.rmtree(bag_filepath)  # Remove the existing directory

    record_cmd = [
        'ros2', 'bag', 'record', '-o', bag_filepath,
        '/joint_states', '/clock', '/odom', '/cmd_vel'
    ]

    rclpy.init(args=None)
    clock_listener = ClockListener()

    # Wait until the clock listener receives a message
    while clock_listener.current_time == 0.0:
        rclpy.spin_once(clock_listener, timeout_sec=0.005)

    start_time = clock_listener.current_time
    print(f"start_time: {start_time}")

    # Start recording process
    print("Starting recording process")
    record_process = subprocess.Popen(
        record_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

    # Start publishing command
    clock_listener.start_publishing_command(x, y, z)
    print("Started publishing command via timer")

    try:
        while clock_listener.current_time - start_time < secs:
            rclpy.spin_once(clock_listener, timeout_sec=0.005)
    finally:
        # Stop publishing command
        clock_listener.stop_publishing_command()

        # Stop the recording process
        os.killpg(os.getpgid(record_process.pid), signal.SIGINT)
        record_process.wait()
        print("Stopped recording process")

        # Send stop command to stop the robot (0 velocity)
        clock_listener.publish_stop_cmd()

        # Wait for 2 seconds of simulation time
        wait_start_time = clock_listener.current_time
        while clock_listener.current_time - wait_start_time < 2:
            rclpy.spin_once(clock_listener, timeout_sec=0.1)

        # Shutdown ROS 2
        clock_listener.destroy_node()
        rclpy.shutdown()

        extract_cmd = [
            'python3', 'extract_wheel_data_from_ros2bag.py',
            'recordings/360/bags', 'recordings/360/wheel_velocities'
        ]
        extract_process = subprocess.run(extract_cmd)

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("Usage: python script.py <X> <Y> <Z> <secs>")
        sys.exit(1)
    X = float(sys.argv[1])
    Y = float(sys.argv[2])
    Z = float(sys.argv[3])
    secs = float(sys.argv[4])
    main(X, Y, Z, secs)
