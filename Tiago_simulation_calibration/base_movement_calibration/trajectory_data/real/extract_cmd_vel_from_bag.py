import rosbag
import json
import sys

def extract_cmd_vel_to_json(bag_file, output_json_file):
    cmd_vel_data = []

    # Open the ROS bag file
    try:
        with rosbag.Bag(bag_file, 'r') as bag:
            print(f"Opened bag file: {bag_file}")

            # Get list of topics in the bag file
            topics_info = bag.get_type_and_topic_info().topics
            topics = topics_info.keys()
            print(f"Topics available in the bag file: {topics}")

            # Check if the expected topic is in the bag file
            if '/mobile_base_controller/cmd_vel' not in topics:
                print(f"Topic '/mobile_base_controller/cmd_vel' not found in the bag file.")
                return

            # Get expected message type for the cmd_vel topic
            expected_type = 'geometry_msgs/Twist'
            actual_type = topics_info['/mobile_base_controller/cmd_vel'].msg_type

            if actual_type != expected_type:
                print(f"Message type mismatch. Expected {expected_type} but found {actual_type}")
                return

            # Loop through all messages in the specified topic
            for topic, msg, t in bag.read_messages(topics=['/mobile_base_controller/cmd_vel']):
                # Directly match message type names
                if topic == '/mobile_base_controller/cmd_vel' and actual_type == expected_type:
                    entry = {
                        'time': t.to_sec(),
                        'linear': {
                            'x': msg.linear.x,
                            'y': msg.linear.y,
                            'z': msg.linear.z
                        },
                        'angular': {
                            'x': msg.angular.x,
                            'y': msg.angular.y,
                            'z': msg.angular.z
                        }
                    }
                    cmd_vel_data.append(entry)
                else:
                    print(f"Skipping message from topic {topic} of type {actual_type}")

            print(f"Extracted {len(cmd_vel_data)} messages from '/mobile_base_controller/cmd_vel' topic.")
    
    except Exception as e:
        print(f"Error reading bag file: {e}")
        return

    # Write the extracted data to the output JSON file
    try:
        with open(output_json_file, 'w') as json_file:
            json.dump(cmd_vel_data, json_file, indent=2)
        print(f"Saved extracted data to {output_json_file}")
    except Exception as e:
        print(f"Error writing to JSON file: {e}")

def main():
    if len(sys.argv) < 3:
        print("Usage: python extract_cmd_vel.py <input_rosbag_file> <output_json_file>")
        return

    bag_file = sys.argv[1]
    output_json_file = sys.argv[2]

    extract_cmd_vel_to_json(bag_file, output_json_file)

if __name__ == '__main__':
    main()
