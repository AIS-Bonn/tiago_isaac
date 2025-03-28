import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import os
import subprocess
import sys
import shutil
import time

# Mapping from each joint to its corresponding command topic and controller joints
joint_topics = {
    'arm_left_1_joint': {'command': '/arm_left_controller/command', 'controller_joints': [
        'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint',
        'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint']},
    'arm_left_2_joint': {'command': '/arm_left_controller/command', 'controller_joints': [
        'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint',
        'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint']},
    'arm_left_3_joint': {'command': '/arm_left_controller/command', 'controller_joints': [
        'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint',
        'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint']},
    'arm_left_4_joint': {'command': '/arm_left_controller/command', 'controller_joints': [
        'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint',
        'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint']},
    'arm_left_5_joint': {'command': '/arm_left_controller/command', 'controller_joints': [
        'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint',
        'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint']},
    'arm_left_6_joint': {'command': '/arm_left_controller/command', 'controller_joints': [
        'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint',
        'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint']},
    'arm_left_7_joint': {'command': '/arm_left_controller/command', 'controller_joints': [
        'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint',
        'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint']},
    'arm_right_1_joint': {'command': '/arm_right_controller/command', 'controller_joints': [
        'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint',
        'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']},
    'arm_right_2_joint': {'command': '/arm_right_controller/command', 'controller_joints': [
        'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint',
        'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']},
    'arm_right_3_joint': {'command': '/arm_right_controller/command', 'controller_joints': [
        'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint',
        'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']},
    'arm_right_4_joint': {'command': '/arm_right_controller/command', 'controller_joints': [
        'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint',
        'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']},
    'arm_right_5_joint': {'command': '/arm_right_controller/command', 'controller_joints': [
        'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint',
        'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']},
    'arm_right_6_joint': {'command': '/arm_right_controller/command', 'controller_joints': [
        'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint',
        'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']},
    'arm_right_7_joint': {'command': '/arm_right_controller/command', 'controller_joints': [
        'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint',
        'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']},
    'gripper_left_left_finger_joint': {'command': '/gripper_left_controller/command', 'controller_joints': [
        'gripper_left_left_finger_joint', 'gripper_left_right_finger_joint']},
    'gripper_left_right_finger_joint': {'command': '/gripper_left_controller/command', 'controller_joints': [
        'gripper_left_left_finger_joint', 'gripper_left_right_finger_joint']},
    'gripper_right_left_finger_joint': {'command': '/gripper_right_controller/command', 'controller_joints': [
        'gripper_right_left_finger_joint', 'gripper_right_right_finger_joint']},
    'gripper_right_right_finger_joint': {'command': '/gripper_right_controller/command', 'controller_joints': [
        'gripper_right_left_finger_joint', 'gripper_right_right_finger_joint']},
    'head_1_joint': {'command': '/head_controller/command', 'controller_joints': [
        'head_1_joint', 'head_2_joint']},
    'head_2_joint': {'command': '/head_controller/command', 'controller_joints': [
        'head_1_joint', 'head_2_joint']},
    'torso_lift_joint': {'command': '/torso_controller/command', 'controller_joints': [
        'torso_lift_joint']}
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

class JointCommandPublisher:
    def __init__(self, joint_name, command_file, recording_folder):

        self.joint_name = joint_name
        self.command_file = command_file
        self.recording_folder = recording_folder
        self.current_joint_positions = {}  # To store the latest positions

        # Subscribe to the /joint_states topic to get the current joint position
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        # Create a persistent publisher
        self.publisher = rospy.Publisher(joint_topics[joint_name]['command'], JointTrajectory, queue_size=10)

        # Wait for the publisher to connect
        rospy.loginfo(f"Waiting for publisher to connect to {joint_topics[joint_name]['command']}...")
        while self.publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def joint_state_callback(self, msg):
        # Update the current joint positions dictionary with the latest values
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def get_current_position(self):
        # Return the latest known position of the target joint
        return self.current_joint_positions.get(self.joint_name, 0.0)

    def wait_for_time(self, duration):
        # Wait for the specified duration using system time
        start_time = time.time()
        while not rospy.is_shutdown() and (time.time() - start_time < duration):
            rospy.sleep(0.1)

    def send_joint_command_with_velocity(self, joint_name, end_pos):
        # Ensure the command is within joint limits
        if joint_name in joint_limits:
            min_limit, max_limit = joint_limits[joint_name]
            if not (min_limit <= end_pos <= max_limit):
                rospy.logwarn(f"Desired position {end_pos} for joint {joint_name} is out of limits ({min_limit}, {max_limit}).")
                return

        # Get the current position of the joint
        start_pos = self.get_current_position()
        rospy.loginfo(f"Moving joint {joint_name} from {start_pos} to {end_pos}")

        # Create and publish the JointTrajectory message
        try:
            msg = JointTrajectory()
            msg.header.stamp = rospy.Time.now()  # Add current time to header
            controller_joints = joint_topics[joint_name]['controller_joints']
            msg.joint_names = controller_joints

            point = JointTrajectoryPoint()

            # Set positions for all joints in the controller, with only the target joint moving
            point.positions = [end_pos if j == joint_name else 0.0 for j in controller_joints]
            point.velocities = [0.0 for _ in controller_joints]
            point.accelerations = [0.0 for _ in controller_joints]
            point.time_from_start = rospy.Duration(3)  # Set sufficient time for the movement

            msg.points = [point]
            self.publisher.publish(msg)
            rospy.loginfo(f"Published joint command to {joint_topics[joint_name]['command']}")

        except Exception as e:
            rospy.logerr(f"Error publishing joint trajectory: {e}")

    def execute_trajectory(self):
        # Ensure the recording folder exists
        os.makedirs(self.recording_folder, exist_ok=True)

        # Read commands from the specified file
        with open(self.command_file, 'r') as file:
            lines = file.readlines()

        # Process each line in the command file
        for line in lines:
            L, R = map(float, line.strip().split())
            rospy.loginfo(f"Executing command: Move from {L} to {R}")

            # Move to initial position L
            self.send_joint_command_with_velocity(self.joint_name, L)
            self.wait_for_time(6)  # Wait to reach initial position

            # Start ROS1 recording
            recording_filename = f"{self.joint_name}/bags/{L}_{R}".replace(" ", "_") + ".bag"
            recording_path = os.path.join(self.recording_folder, recording_filename)
            if os.path.exists(recording_path):
                shutil.rmtree(recording_path)

            # Ensure the parent directory exists for the recording file
            os.makedirs(os.path.dirname(recording_path), exist_ok=True)

            recording_process = subprocess.Popen(["rosbag", "record", "-O", recording_path,
                                                  "/joint_states", joint_topics[self.joint_name]['command']])
            rospy.loginfo(f"Recording started: {recording_filename}")

            # Short delay to ensure recording is ready
            self.wait_for_time(2)

            self.send_joint_command_with_velocity(self.joint_name, R)
            self.wait_for_time(4)

            # Stop recording after command is completed
            recording_process.terminate()
            rospy.loginfo(f"Recording stopped: {recording_filename}")
            self.wait_for_time(2)

            self.send_joint_command_with_velocity(self.joint_name, 0.0)  # Reset joint position to 0
            self.wait_for_time(4)

def main():
    # Retrieve command file and recording folder from command-line arguments
    if len(sys.argv) < 3:
        print("Usage: python joint_command_publisher.py <command_file> <recording_folder>")
        return

    command_file = sys.argv[1]
    recording_folder = sys.argv[2]

    # Extract joint name from the command file name (assumes format '<joint_name>.txt')
    joint_name = os.path.splitext(os.path.basename(command_file))[0]

    if joint_name not in joint_topics:
        rospy.logerr(f"Joint {joint_name} is not in the list of known joint topics.")
        return

    rospy.init_node('recording_joint_command_publisher', anonymous=True)

    joint_command_publisher = JointCommandPublisher(joint_name, command_file, recording_folder)
    joint_command_publisher.execute_trajectory()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
