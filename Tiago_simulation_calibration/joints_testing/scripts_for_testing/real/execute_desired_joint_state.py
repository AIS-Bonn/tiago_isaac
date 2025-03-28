import rospy
import os
import yaml
import threading
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist

# Mapping from each joint to its corresponding command topic
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

# Define min and max values for each joint
joint_limits = {
    'arm_left_1_joint': (-1.1780972450961724, 1.5707963267948966),
    'arm_left_2_joint': (-1.1780972450961724, 1.5707963267948966),
    'arm_left_3_joint': (-0.7853981633974483, 3.9269908169872414),
    'arm_left_4_joint': (-0.39269908169872414, 2.356194490192345),
    'arm_left_5_joint': (-2.0943951023931953, 2.0943951023931953),
    'arm_left_6_joint': (-1.413716694115407, 1.413716694115407),
    'arm_left_7_joint': (-2.0943951023931953, 2.0943951023931953),
    'arm_right_1_joint': (-1.1780972450961724, 1.5707963267948966),
    'arm_right_2_joint': (-1.1780972450961724, 1.5707963267948966),
    'arm_right_3_joint': (-0.7853981633974483, 3.9269908169872414),
    'arm_right_4_joint': (-0.39269908169872414, 2.356194490192345),
    'arm_right_5_joint': (-2.0943951023931953, 2.0943951023931953),
    'arm_right_6_joint': (-1.413716694115407, 1.413716694115407),
    'arm_right_7_joint': (-2.0943951023931953, 2.0943951023931953),
    'gripper_left_left_finger_joint': (-0.02, 0.045),
    'gripper_left_right_finger_joint': (-0.02, 0.045),
    'gripper_right_left_finger_joint': (0.0, 0.045),
    'gripper_right_right_finger_joint': (0.0, 0.045),
    'head_1_joint': (-1.3089969389957472, 1.3089969389957472),
    'head_2_joint': (-1.0471975511965976, 0.7853981633974483),
    'torso_lift_joint': (0.0, 0.35)
}

# Dictionary to store publishers for each controller
defined_publishers = {}

def initialize_publishers():
    global defined_publishers
    for joint, topic_info in joint_topics.items():
        controller_topic = topic_info['command']
        if controller_topic not in defined_publishers:
            defined_publishers[controller_topic] = rospy.Publisher(controller_topic, JointTrajectory, queue_size=10)

    # Wait for the publisher to establish connection
    rospy.loginfo("Waiting for publishers to connect...")
    for publisher in defined_publishers.values():
        rospy.loginfo(f"Waiting for publisher to connect to {controller_topic}...")
        while publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)


def read_desired_joint_states(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        rospy.logwarn(f"File {file_path} not found.")
        return None
    except Exception as e:
        rospy.logerr(f"Error reading file {file_path}: {e}")
        return None

def write_current_joint_states(file_path, joint_states):
    try:
        with open(file_path, 'w') as file:
            yaml.dump(joint_states, file, default_flow_style=False)
        rospy.loginfo(f"Current joint states written to {file_path}.")
    except Exception as e:
        rospy.logerr(f"Error writing file {file_path}: {e}")

def publish_joint_trajectory(joint_states):
    if not joint_states:
        return

    controllers = {}

    # Group joint states by their corresponding controller
    for joint, position in joint_states.items():
        controller_topic = joint_topics[joint]['command']
        if controller_topic not in controllers:
            controllers[controller_topic] = {}
        controllers[controller_topic][joint] = position
        print(position)

    # Publish trajectories for each controller
    for controller_topic, joint_group in controllers.items():
        publisher = defined_publishers[controller_topic]


        try:
            traj = JointTrajectory()
            traj.header.stamp = rospy.Time.now()
            traj.joint_names = list(joint_group.keys())

            point = JointTrajectoryPoint()
            point.positions = list(joint_group.values())
            point.velocities = [0.0] * len(joint_group)
            point.accelerations = [0.0] * len(joint_group)
            point.time_from_start = rospy.Duration(2.0)  # Increased time for smoother motion

            traj.points = [point]

            publisher.publish(traj)
            rospy.loginfo(f"Published joint trajectory to {controller_topic}.")
            print(traj)

        except Exception as e:
            rospy.logerr(f"Error publishing joint trajectory to {controller_topic}: {e}")

def read_desired_base_movement(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        # rospy.logwarn(f"File {file_path} not found.")
        return None
    except Exception as e:
        rospy.logerr(f"Error reading file {file_path}: {e}")
        return None

def publish_base_movement():
    publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        # Read the desired base movement command from file
        twist_command = read_desired_base_movement(base_movement_file_path)
        if not twist_command:
            rospy.sleep(0.01)
            continue

        # Wait for the publisher to establish connection
        rospy.loginfo("Waiting for publisher to connect to /mobile_base_controller/cmd_vel...")
        while publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.01)

        try:
            twist = Twist()
            twist.linear.x = twist_command['linear']['x']
            twist.linear.y = twist_command['linear']['y']
            twist.linear.z = twist_command['linear']['z']
            twist.angular.x = twist_command['angular']['x']
            twist.angular.y = twist_command['angular']['y']
            twist.angular.z = twist_command['angular']['z']

            publisher.publish(twist)
            rospy.loginfo("Published base movement command.")

        except Exception as e:
            rospy.logerr(f"Error publishing base movement command: {e}")

        rospy.sleep(0.01)  # Publish at 10 Hz

def publish_joint_trajectory_periodically():
    rate = rospy.Rate(1)  # 1 Hz for joint trajectory updates
    while not rospy.is_shutdown():
        joint_states = read_desired_joint_states(desired_file_path)
        if joint_states:
            publish_joint_trajectory(joint_states)

        # Simulate fetching current joint states (Replace with actual fetch logic if available)
        current_joint_states = {joint: position for joint, position in joint_states.items()}  # Simulate same as desired
        write_current_joint_states(current_file_path, current_joint_states)

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('desired_joint_reader', anonymous=True)

    desired_file_path = '/home/pal/desired_joint_states.txt'
    current_file_path = '/home/pal/current_joint_angles.txt'
    base_movement_file_path = '/home/pal/desired_base_movement.txt'

    # Initialize publishers
    initialize_publishers()

    # Start joint trajectory publishing in a separate thread
    joint_thread = threading.Thread(target=publish_joint_trajectory_periodically)
    joint_thread.start()

    # Start base movement publishing in the main thread
    publish_base_movement()
