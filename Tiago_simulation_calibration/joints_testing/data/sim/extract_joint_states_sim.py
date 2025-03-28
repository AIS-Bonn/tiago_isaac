import os
import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
import rclpy

# Initialize ROS2 node (required to deserialize messages)
rclpy.init(args=None)

def interpolate_clock(clock_data, target_timestamp):
    """
    Interpolates simulation time based on /clock data for a given target system timestamp.
    """
    for i in range(len(clock_data) - 1):
        t0, sim_time0 = clock_data[i]
        t1, sim_time1 = clock_data[i + 1]
        if t0 <= target_timestamp <= t1:
            # Linear interpolation
            alpha = (target_timestamp - t0) / (t1 - t0)
            interpolated_sim_time = sim_time0 + alpha * (sim_time1 - sim_time0)
            return interpolated_sim_time
    return None  # Return None if no interpolation is possible

def get_start_timestamp(bag_path):
    """
    Extracts the timestamp of the first /joint_command message.
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    # Loop through messages to find the first /joint_command message
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == '/joint_command':
            return timestamp  # Return the timestamp of the first message

    return None  # Return None if no /joint_command message is found

def process_bag_file(bag_path, joint_name, output_csv_path):
    # Open the bag
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    # Collect /clock data
    clock_data = []
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == '/clock':
            clock_msg = deserialize_message(data, Clock)
            sim_time = clock_msg.clock.sec + clock_msg.clock.nanosec * 1e-9
            clock_data.append((timestamp, sim_time))

    # Get the start timestamp from /joint_command
    T_start = get_start_timestamp(bag_path)
    if T_start is None:
        print(f"No /joint_command messages found in {bag_path}. Skipping.")
        return

    # Reset reader to process /joint_states
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    with open(output_csv_path, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['simulation_time', 'position', 'velocity', 'effort'])  # Header

        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()

            if topic == '/joint_states' and timestamp >= T_start:
                sim_time = interpolate_clock(clock_data, timestamp)
                if sim_time is None:
                    continue  # Skip if no corresponding simulation time is found

                msg = deserialize_message(data, JointState)
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    position = msg.position[idx] if idx < len(msg.position) else None
                    velocity = msg.velocity[idx] if idx < len(msg.velocity) else None
                    effort = msg.effort[idx] if idx < len(msg.effort) else None

                    csv_writer.writerow([sim_time, position, velocity, effort])

# Define the main function to process all bag files
def main():
    recordings_dir = 'recordings'

    for joint_name in os.listdir(recordings_dir):
        joint_dir = os.path.join(recordings_dir, joint_name)
        print(f"Processing joint {joint_name}")

        if os.path.isdir(joint_dir):
            bags_dir = os.path.join(joint_dir, 'bags')
            poses_dir = os.path.join(joint_dir, 'poses')
            os.makedirs(poses_dir, exist_ok=True)

            for bag_folder in os.listdir(bags_dir):
                bag_path = os.path.join(bags_dir, bag_folder)

                if os.path.isdir(bag_path):
                    bag_folder = bag_folder.replace(".bag", "")
                    output_csv_path = os.path.join(poses_dir, f"{bag_folder}.csv") 

                    process_bag_file(bag_path, joint_name, output_csv_path)
                    print(f"Processed {bag_folder} for joint {joint_name}")

    print("All ROS2 bag files have been processed.")

# Run the main function
if __name__ == '__main__':
    main()
