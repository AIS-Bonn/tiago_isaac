#!/usr/bin/env python3

import os
import csv
import rclpy
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import sqlite3
import numpy as np

# Initialize the ROS 2 Python client library
rclpy.init()

# Define the wheel joint names
wheel_joints = [
    "wheel_front_left_joint",
    "wheel_front_right_joint",
    "wheel_rear_left_joint",
    "wheel_rear_right_joint"
]

# Check if the Twist message contains all zeros
def is_zero_twist(msg):
    return (msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and
            msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0)

# Check if two Twist messages are different
def twist_is_different(twist1, twist2):
    return (twist1.linear.x != twist2.linear.x or twist1.linear.y != twist2.linear.y or twist1.linear.z != twist2.linear.z or
            twist1.angular.x != twist2.angular.x or twist1.angular.y != twist2.angular.y or twist1.angular.z != twist2.angular.z)

# Extract and interpolate /clock time data
def extract_clock_times(cursor):
    clock_times, sim_times = [], []
    cursor.execute("""
        SELECT timestamp, data 
        FROM messages 
        WHERE topic_id = (
            SELECT id 
            FROM topics 
            WHERE name = '/clock'
        )
    """)
    for row in cursor.fetchall():
        timestamp, data = row
        clock_msg = deserialize_message(data, get_message('rosgraph_msgs/msg/Clock'))
        sim_time = clock_msg.clock.sec + clock_msg.clock.nanosec * 1e-9
        clock_times.append(timestamp)
        sim_times.append(sim_time)
    return np.array(clock_times), np.array(sim_times)

# Process ROS 2 bag files and split based on twist command changes
def process_ros2_bag(bag_path, csv_path_base):
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        print(f"No .db3 file found in {bag_path}. Skipping.")
        return
    db_file = db_files[0]
    db_path = os.path.join(bag_path, db_file)

    # Connect to the SQLite3 database
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Ensure the table exists and fetch topic IDs for /joint_states, /cmd_vel, and /clock
    cursor.execute("SELECT id FROM topics WHERE name = '/joint_states'")
    joint_states_topic_id = cursor.fetchone()[0]

    cursor.execute("SELECT id FROM topics WHERE name = '/cmd_vel'")
    twist_topic_result = cursor.fetchone()

    if twist_topic_result is None:
        print(f"No '/cmd_vel' topic found in {bag_path}. Skipping.")
        conn.close()
        return

    twist_topic_id = twist_topic_result[0]

    # Extract clock times for interpolation
    clock_times, sim_times = extract_clock_times(cursor)

    # Find the first non-zero twist command and timestamp
    cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {twist_topic_id}")
    twist_msgs = cursor.fetchall()

    initial_twist = None
    start_time = None

    for timestamp, data in twist_msgs:
        twist_msg = deserialize_message(data, get_message('geometry_msgs/msg/Twist'))
        if not is_zero_twist(twist_msg):
            initial_twist = twist_msg
            start_time = timestamp
            break

    if not initial_twist:
        print("No non-zero twist found. Skipping.")
        conn.close()
        return

    segment_count = 0
    current_start_time = start_time
    previous_twist = initial_twist
    for timestamp, data in twist_msgs:
        twist_msg = deserialize_message(data, get_message('geometry_msgs/msg/Twist'))
        if timestamp > current_start_time and twist_is_different(previous_twist, twist_msg):
            if segment_count == 0:
                process_segment(db_path, csv_path_base, joint_states_topic_id, current_start_time, timestamp, clock_times, sim_times)
            elif segment_count == 1:
                process_segment(db_path, f"{csv_path_base}_reverse", joint_states_topic_id, current_start_time, timestamp, clock_times, sim_times)
            else:
                break
            current_start_time = timestamp
            previous_twist = twist_msg
            segment_count += 1

    if segment_count == 0:
        process_segment(db_path, csv_path_base, joint_states_topic_id, current_start_time, None, clock_times, sim_times)
    
    conn.close()
    print(f"Processed {bag_path} and saved to {csv_path_base}")

# Update process_segment to use interpolated /clock time
def process_segment(db_path, csv_path_base, joint_states_topic_id, start_time, end_time, clock_times, sim_times):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    csv_file = f"{csv_path_base}.csv"
    
    with open(csv_file, 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time', 'Front Left', 'Front Right', 'Rear Left', 'Rear Right'])

        cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {joint_states_topic_id}")
        for timestamp, data in cursor.fetchall():
            if start_time <= timestamp and (end_time is None or timestamp < end_time):
                sim_time = np.interp(timestamp, clock_times, sim_times)
                msg = deserialize_message(data, get_message('sensor_msgs/msg/JointState'))
                wheel_indices = [msg.name.index(joint) for joint in wheel_joints]
                wheel_velocities = [-msg.velocity[i] for i in wheel_indices] if "reverse" in csv_file else [msg.velocity[i] for i in wheel_indices]
                writer.writerow([sim_time] + wheel_velocities)

    conn.close()
    print(f"Segment processed and saved to {csv_file}")

def process_all_bag_folders(bags_folder, wheel_velocities_folder):
    for bag_folder in os.listdir(bags_folder):
        if os.path.isdir(os.path.join(bags_folder, bag_folder)):
            bag_path = os.path.join(bags_folder, bag_folder)
            csv_path_base = os.path.join(wheel_velocities_folder, bag_folder.split('.bag')[0])
            print(f"Processing {bag_path}")
            process_ros2_bag(bag_path, csv_path_base)

    rclpy.shutdown()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python script.py bags_folder output_folder")
        sys.exit(1)
    bags_folder = sys.argv[1]
    wheel_velocities_folder = sys.argv[2]
    os.makedirs(bags_folder, exist_ok=True)
    os.makedirs(wheel_velocities_folder, exist_ok=True)
    process_all_bag_folders(bags_folder, wheel_velocities_folder)
