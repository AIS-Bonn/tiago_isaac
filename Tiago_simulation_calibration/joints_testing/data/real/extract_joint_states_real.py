import os
import csv
import rosbag
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

joint_topics = {
    'arm_left_1_joint': {'command': '/arm_left_controller/command'},
    'arm_left_2_joint': {'command': '/arm_left_controller/command'},
    'arm_left_3_joint': {'command': '/arm_left_controller/command'},
    'arm_left_4_joint': {'command': '/arm_left_controller/command'},
    'arm_left_5_joint': {'command': '/arm_left_controller/command'},
    'arm_left_6_joint': {'command': '/arm_left_controller/command'},
    'arm_left_7_joint': {'command': '/arm_left_controller/command'},
    'arm_right_1_joint': {'command': '/arm_right_controller/command'},
    'arm_right_2_joint': {'command': '/arm_right_controller/command'},
    'arm_right_3_joint': {'command': '/arm_right_controller/command'},
    'arm_right_4_joint': {'command': '/arm_right_controller/command'},
    'arm_right_5_joint': {'command': '/arm_right_controller/command'},
    'arm_right_6_joint': {'command': '/arm_right_controller/command'},
    'arm_right_7_joint': {'command': '/arm_right_controller/command'},
    'gripper_left_left_finger_joint': {'command': '/gripper_left_controller/command'},
    'gripper_left_right_finger_joint': {'command': '/gripper_left_controller/command'},
    'gripper_right_left_finger_joint': {'command': '/gripper_right_controller/command'},
    'gripper_right_right_finger_joint': {'command': '/gripper_right_controller/command'},
    'head_1_joint': {'command': '/head_controller/command'},
    'head_2_joint': {'command': '/head_controller/command'},
    'torso_lift_joint': {'command': '/torso_controller/command'}
}

def get_first_command_timestamp(bag, command_topic):
    """
    Retrieves the timestamp of the first message on the command topic.
    """
    for topic, msg, timestamp in bag.read_messages(topics=[command_topic]):
        return timestamp.to_sec()
    return None

def process_bag_file(bag_path, joint_name, output_csv_path):
    """
    Processes a ROS1 bag file to extract joint state data for a specific joint
    and writes the data to a CSV file.
    """
    command_topic = joint_topics[joint_name]['command']

    # Open the ROS1 bag file
    with rosbag.Bag(bag_path, 'r') as bag:
        # Get the first command timestamp
        first_command_timestamp = get_first_command_timestamp(bag, command_topic)
        if first_command_timestamp is None:
            print(f"No commands found for joint {joint_name} in {bag_path}")
            return

        # Open CSV file for writing
        with open(output_csv_path, mode='w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(['timestamp', 'position', 'velocity', 'effort'])  # Header

            # Iterate through messages in the bag file
            for topic, msg, timestamp in bag.read_messages(topics=['/joint_states']):
                if topic == '/joint_states' and timestamp.to_sec() >= first_command_timestamp:
                    # Find the joint of interest
                    if joint_name in msg.name:
                        idx = msg.name.index(joint_name)
                        position = msg.position[idx] if idx < len(msg.position) else None
                        velocity = msg.velocity[idx] if idx < len(msg.velocity) else None
                        effort = msg.effort[idx] if idx < len(msg.effort) else None

                        # Write data to CSV
                        csv_writer.writerow([timestamp.to_sec(), position, velocity, effort])

def main():
    """
    Main function to process all ROS1 bag files in the "recordings" directory.
    """
    recordings_dir = 'recordings3sec'

    # Loop through each subfolder (joint name) in recordings
    for joint_name in os.listdir(recordings_dir):
        joint_dir = os.path.join(recordings_dir, joint_name)
        print(f"Processing joint {joint_name}")

        # Check if it's a directory
        if os.path.isdir(joint_dir):
            bags_dir = os.path.join(joint_dir, 'bags')
            poses_dir = os.path.join(joint_dir, 'poses')
            os.makedirs(poses_dir, exist_ok=True)  # Create poses directory if it doesn't exist

            # Loop through each bag file in the bags directory
            for bag_file in os.listdir(bags_dir):
                bag_path = os.path.join(bags_dir, bag_file)

                # Check if it's a ROS1 bag file
                if bag_file.endswith('.bag'):
                    # Define the output CSV path
                    output_csv_path = os.path.join(poses_dir, f"{os.path.splitext(bag_file)[0]}.csv")

                    # Process the bag file and write data to CSV
                    process_bag_file(bag_path, joint_name, output_csv_path)
                    print(f"Processed {bag_file} for joint {joint_name}")

    print("All ROS1 bag files have been processed.")

if __name__ == '__main__':
    main()
