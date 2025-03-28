#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock  # Correct import
import subprocess
import sys
import os

# Define a ClockSubscriber class to listen to the /clock topic
class ClockSubscriber(Node):
    def __init__(self):
        super().__init__('clock_subscriber')
        self.subscription = self.create_subscription(
            Clock,  # Correct message type
            '/clock',
            self.clock_callback,
            10)
        self.simulation_time = None

    def clock_callback(self, msg):
        # Store the latest clock time in seconds
        self.simulation_time = msg.clock.sec + msg.clock.nanosec * 1e-9

# Define the function to execute the command
def execute_command(speed, secs, clock_subscriber, folder):
    command = ["python3", "record_square_trajectory_sim.py", str(speed), str(secs), str(folder)]
    print(f"Executing: {' '.join(command)}")
    subprocess.run(command)

    # Wait for secs amount of ROS2 time using the /clock topic
    start_time = clock_subscriber.simulation_time
    while clock_subscriber.simulation_time is None or clock_subscriber.simulation_time - start_time < secs + 3:
        rclpy.spin_once(clock_subscriber)

    # Optionally reset the robot back (commented for now)
    # reset_command = ["python3", "record_simple_trajectory_real.py", str(-x), str(-y), str(-z), str(secs)]
    # print(f"Executing reset: {' '.join(reset_command)}")
    # subprocess.run(reset_command)

    # Wait an additional 1 seconds of simulation time after the reset
    start_time = clock_subscriber.simulation_time
    while clock_subscriber.simulation_time is None or clock_subscriber.simulation_time - start_time < secs + 1:
        rclpy.spin_once(clock_subscriber)

# Function to read the file and execute commands
def process_file(file_path, clock_subscriber):
    # Get the base name of the file (e.g., '0.5xy.txt')
    base_name = os.path.basename(file_path)
    name, extension = os.path.splitext(base_name)

    with open(file_path, "r") as file:
        for line in file:
            # Skip empty lines and lines starting with #
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            # Parse the line to get X, Y, Z, SECS
            try:
                speed, traj = map(float, line.split())
                execute_command(speed, traj, clock_subscriber, folder=name+"/bags")
            except ValueError:
                print(f"Invalid line format: {line}")

# Main function to run the script
def main():
    if len(sys.argv) != 2:
        print("Usage: python script.py command_file")
        sys.exit(1)
    commands_file = sys.argv[1]

    # Set ROS_DOMAIN_ID to 68
    os.environ['ROS_DOMAIN_ID'] = '68'
    rclpy.init()  # Initialize the ROS2 Python interface

    clock_subscriber = ClockSubscriber()  # Create the clock subscriber node

    # Wait until we receive the first clock time before proceeding
    while clock_subscriber.simulation_time is None:
        rclpy.spin_once(clock_subscriber)

    try:
        process_file(commands_file, clock_subscriber)  # Process the command file
    finally:
        clock_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
