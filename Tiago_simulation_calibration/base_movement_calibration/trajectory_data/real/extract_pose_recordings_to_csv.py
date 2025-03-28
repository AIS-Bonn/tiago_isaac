import os
import rosbag
import pandas as pd
import sys
from geometry_msgs.msg import Twist

def is_zero_twist(msg):
    """Check if the Twist message contains all zeros."""
    return (msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and
            msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0)

def twist_is_different(twist1, twist2):
    """Check if two Twist messages are different."""
    return (twist1.linear.x != twist2.linear.x or twist1.linear.y != twist2.linear.y or twist1.linear.z != twist2.linear.z or
            twist1.angular.x != twist2.angular.x or twist1.angular.y != twist2.angular.y or twist1.angular.z != twist2.angular.z)

def find_first_non_zero_twist(bag):
    """Find the first non-zero Twist message."""
    for topic, msg, t in bag.read_messages(topics=['/mobile_base_controller/cmd_vel']):
        if not is_zero_twist(msg):
            return msg, t.to_sec()  # Return the first non-zero twist and 
        # else:
        #     print(f"Found zero twist at {t.to_sec()} s")
    return None, None

def process_pose_file(bag_path, session_df, pose_recordings_folder, start_time, end_time, reverse_suffix=False):
    """Process pose data between start_time and end_time, and write it to a CSV file."""
    # Set the output CSV file name with or without the reverse suffix
    csv_file = os.path.basename(bag_path).replace('.bag', '_reverse.csv' if reverse_suffix else '.csv')
    output_csv_path = os.path.join(pose_recordings_folder, csv_file)

    # Filter the session_df DataFrame based on the timestamps start_time and end_time
    subset_df = session_df[(session_df['timestamp'] >= start_time * 1000) & (session_df['timestamp'] <= end_time * 1000)]  # Convert to milliseconds

    if not subset_df.empty:
        subset_df.to_csv(output_csv_path, index=False)
        print(f"Extracted pose data from {start_time} to {end_time} into {output_csv_path}")
    else:
        print(f"No pose data found between {start_time} and {end_time}.")

def process_all_bag_folders(session_csv_path, bag_files_folder, pose_recordings_folder):
    # Load the session.csv into a DataFrame
    session_df = pd.read_csv(session_csv_path)

    # Iterate through each .bag file in the folder
    for bag_file in os.listdir(bag_files_folder):
        if bag_file.endswith('.bag'):
            bag_file_path = os.path.join(bag_files_folder, bag_file)

            print(f"Processing {bag_file}")
            # Open the .bag file
            with rosbag.Bag(bag_file_path) as bag:
                for topic, msg, t in bag.read_messages(topics=['/mobile_base_controller/cmd_vel']):
                    initial_time = t.to_sec()
                    break
                # Find the first non-zero twist command
                initial_twist, start_time = find_first_non_zero_twist(bag)
                print(f"Initial twist: {initial_twist}, Start time: {start_time}")
                if initial_twist is None:
                    print(f"No non-zero twist found in {bag_file}. Skipping.")
                    continue  # Skip this bag if no non-zero twist was found
                
                print(f"First non-zero twist found at {start_time} s, which is {start_time-initial_time} s after the start of the recording.")

                # Variables to track the start and end times of twist commands
                current_start_time = start_time
                previous_twist = initial_twist

                next_is_reversed = False

                # Iterate through the /mobile_base_controller/cmd_vel messages to detect twist changes
                for topic, msg, t in bag.read_messages(topics=['/mobile_base_controller/cmd_vel']):
                    current_time = t.to_sec()

                    if current_time > current_start_time:
                        # If the Twist command changes, process the interval
                        if twist_is_different(previous_twist, msg) and not is_zero_twist(msg):
                            print(f"Twist change detected in {bag_file} at {current_time} s. Processing new pose file.")
                            print(f"Previous twist: {previous_twist}, New twist: {msg}")
                            # Process the previous segment
                            process_pose_file(bag_file_path, session_df, pose_recordings_folder, current_start_time, end_time=current_time, reverse_suffix=next_is_reversed)

                            # Update the start time for the new command
                            current_start_time = current_time
                            previous_twist = msg

                            if next_is_reversed:
                                break

                            next_is_reversed = True


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python script.py session_csv_path bags_folder output_folder")
        sys.exit(1)
    session_csv_path = sys.argv[1]
    bags_folder = sys.argv[2]
    pose_recording_folder = sys.argv[3]

    os.makedirs(bags_folder, exist_ok=True)
    os.makedirs(pose_recording_folder, exist_ok=True)
    process_all_bag_folders(session_csv_path, bags_folder, pose_recording_folder)
