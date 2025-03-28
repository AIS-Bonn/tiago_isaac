import os
import sqlite3
import csv
import sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def is_zero_twist(msg):
    return (msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and
            msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0)

def twist_is_different(twist1, twist2):
    return (twist1.linear.x != twist2.linear.x or twist1.linear.y != twist2.linear.y or twist1.linear.z != twist2.linear.z or
            twist1.angular.x != twist2.angular.x or twist1.angular.y != twist2.angular.y or twist1.angular.z != twist2.angular.z)

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

def extract_odom_segment(db3_file, output_file, odom_topic_id, clock_times, sim_times, start_time, end_time):
    conn = sqlite3.connect(db3_file)
    cursor = conn.cursor()
    odom_msg_type = get_message('nav_msgs/msg/Odometry')

    with open(output_file, 'w', newline='') as csvfile:
        fieldnames = ['timestamp', 'pos_x', 'pos_y', 'pos_z', 
                      'rot_00', 'rot_01', 'rot_02', 
                      'rot_10', 'rot_11', 'rot_12', 
                      'rot_20', 'rot_21', 'rot_22']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        cursor.execute(f"""
            SELECT timestamp, data 
            FROM messages 
            WHERE topic_id = {odom_topic_id} 
            AND timestamp >= {start_time} 
            AND ({end_time} IS NULL OR timestamp < {end_time})
        """)

        for row in cursor.fetchall():
            timestamp = row[0]
            sim_time = np.interp(timestamp, clock_times, sim_times)
            msg = deserialize_message(row[1], odom_msg_type)

            pos_x, pos_y, pos_z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
            qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
            rot_00, rot_01, rot_02 = 1 - 2 * (qy**2 + qz**2), 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy)
            rot_10, rot_11, rot_12 = 2 * (qx*qy + qw*qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy*qz - qw*qx)
            rot_20, rot_21, rot_22 = 2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx), 1 - 2 * (qx**2 + qy**2)

            writer.writerow({'timestamp': sim_time, 'pos_x': pos_x, 'pos_y': pos_y, 'pos_z': pos_z,
                             'rot_00': rot_00, 'rot_01': rot_01, 'rot_02': rot_02,
                             'rot_10': rot_10, 'rot_11': rot_11, 'rot_12': rot_12,
                             'rot_20': rot_20, 'rot_21': rot_21, 'rot_22': rot_22})
    conn.close()

def process_db_with_twist(db3_file, output_file_base):
    conn = sqlite3.connect(db3_file)
    cursor = conn.cursor()

    cursor.execute("SELECT id FROM topics WHERE name = '/odom'")
    odom_topic_id = cursor.fetchone()[0]

    cursor.execute("SELECT id FROM topics WHERE name = '/cmd_vel'")
    twist_topic_result = cursor.fetchone()

    if twist_topic_result is None:
        print(f"No '/cmd_vel' topic found in {db3_file}. Skipping.")
        conn.close()
        return

    twist_topic_id = twist_topic_result[0]
    clock_times, sim_times = extract_clock_times(cursor)
    twist_msgs = cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {twist_topic_id}").fetchall()

    initial_twist, start_time = None, None
    segment_count = 0

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

    current_start_time = start_time
    previous_twist = initial_twist

    for timestamp, data in twist_msgs:
        twist_msg = deserialize_message(data, get_message('geometry_msgs/msg/Twist'))
        if timestamp > current_start_time and twist_is_different(previous_twist, twist_msg):
            output_file = output_file_base + (".csv" if segment_count == 0 else "_reverse.csv")
            extract_odom_segment(db3_file, output_file, odom_topic_id, clock_times, sim_times, current_start_time, timestamp)
            segment_count += 1

            if segment_count > 1:
                break

            current_start_time = timestamp
            previous_twist = twist_msg

    if segment_count == 0:
        extract_odom_segment(db3_file, output_file_base + ".csv", odom_topic_id, clock_times, sim_times, current_start_time, None)

    conn.close()
    print(f"Processed {db3_file} and saved to {output_file_base}")

def process_all_bag_folders(base_folder, pose_recording_folder):
    for folder_name in os.listdir(base_folder):
        folder_path = os.path.join(base_folder, folder_name)
        
        if os.path.isdir(folder_path):
            db3_file = os.path.join(folder_path, folder_name + '_0.db3')
            folder_namebase = folder_name.split('.bag')[0]
            output_file_base = os.path.join(pose_recording_folder, folder_namebase)

            if os.path.exists(db3_file):
                print(f"Processing {db3_file}")
                process_db_with_twist(db3_file, output_file_base)
            else:
                print(f"No .db3 file found in {folder_path}. Skipping...")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python script.py bags_folder output_folder")
        sys.exit(1)
    bags_folder = sys.argv[1]
    pose_recording_folder = sys.argv[2]

    os.makedirs(pose_recording_folder, exist_ok=True)
    process_all_bag_folders(bags_folder, pose_recording_folder)
