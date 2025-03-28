#!/usr/bin/env python

import rosbag
import csv
import os
import sys
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

# Define the wheel joint names
wheel_joints = [
    "wheel_front_left_joint",
    "wheel_front_right_joint",
    "wheel_rear_left_joint",
    "wheel_rear_right_joint"
]


def is_zero_twist(msg):
    """Check if the Twist message contains all zeros."""
    return (msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and
            msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0)


def twist_is_different(twist1, twist2):
    """Check if two Twist messages are different."""
    return (twist1.linear.x != twist2.linear.x or twist1.linear.y != twist2.linear.y or twist1.linear.z != twist2.linear.z or
            twist1.angular.x != twist2.angular.x or twist1.angular.y != twist2.angular.y or twist1.angular.z != twist2.angular.z)


def find_first_non_zero_twist(bag):
    """Find the first non-zero Twist message and the subsequent changing Twist."""
    for topic, msg, t in bag.read_messages(topics=['/mobile_base_controller/cmd_vel']):
        if not is_zero_twist(msg):
            return msg, t.to_sec()  # Return the first non-zero twist and timestamp
    return None, None


def process_bag_file(bag_path, wheel_velocities_directory, start_time, end_time, reverse_suffix=False):
    """Process a bag file starting from a given start time and write wheel velocity data to a CSV file until the end time."""
    # Set the output CSV file name with or without the reverse suffix
    csv_file = os.path.basename(bag_path).replace('.bag', '_reverse.csv' if reverse_suffix else '.csv')
    csv_path = os.path.join(wheel_velocities_directory, csv_file)

    with rosbag.Bag(bag_path, 'r') as bag, open(csv_path, 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time', 'Front Left', 'Front Right', 'Rear Left', 'Rear Right'])

        # Process JointState messages between start_time and end_time
        for topic, msg, t in bag.read_messages(topics=['/joint_states']):
            current_time = t.to_sec()
            if start_time <= current_time < end_time:
                try:
                    wheel_indices = [msg.name.index(joint) for joint in wheel_joints]
                    wheel_velocities = [msg.velocity[i] for i in wheel_indices]
                    if reverse_suffix:
                        wheel_velocities = [-v for v in wheel_velocities]
                    writer.writerow([current_time] + wheel_velocities)
                except ValueError as e:
                    print(f"Error processing {os.path.basename(bag_path)}: {e}")
                    continue

    print(f"Processed {os.path.basename(bag_path)} from {start_time} to {end_time} and saved to {csv_file}")


def process_all_bag_folders(bag_directory, wheel_velocities_directory):
    # Loop through all .bag files in the specified directory
    for bag_file in os.listdir(bag_directory):
        if bag_file.endswith(".bag"):
            print(f"Processing {bag_file}")
            # Construct full paths for the input .bag file and output .csv file
            bag_path = os.path.join(bag_directory, bag_file)

            # Process the .bag file
            with rosbag.Bag(bag_path, 'r') as bag:
                # Find the first non-zero twist message and its timestamp
                initial_twist, start_time = find_first_non_zero_twist(bag)
                print(f"First non-zero twist found at {start_time} s")

                if initial_twist is None:
                    print(f"No non-zero twist found in {bag_file}. Skipping.")
                    continue

                # Variables to track the start and end times of twist commands
                current_start_time = start_time
                previous_twist = initial_twist

                # Now, look for the change in Twist command after the initial command
                for topic, msg, t in bag.read_messages(topics=['/mobile_base_controller/cmd_vel']):
                    current_time = t.to_sec()
                    if current_time > current_start_time:
                        # If the Twist command changes, close the previous file and start a new one
                        if twist_is_different(previous_twist, msg) and not is_zero_twist(msg):
                            print(f"Twist change detected in {bag_file} at {current_time} s. Ending previous file and starting new.")
                            # Process the previous segment and close it
                            process_bag_file(bag_path, wheel_velocities_directory, current_start_time, current_time)

                            # Update the start time for the new command
                            current_start_time = current_time
                            previous_twist = msg

                            # Now process the new twist command
                            process_bag_file(bag_path, wheel_velocities_directory, current_start_time, float('inf'), reverse_suffix=True)

                            break


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python script.py bags_folder output_folder")
        sys.exit(1)
    bags_folder = sys.argv[1]
    wheel_velocities_directory = sys.argv[2]

    os.makedirs(bags_folder, exist_ok=True)
    os.makedirs(wheel_velocities_directory, exist_ok=True)
    process_all_bag_folders(bags_folder, wheel_velocities_directory)
