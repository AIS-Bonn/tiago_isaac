import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
import os
import subprocess
import sys
from rclpy.parameter import Parameter
import shutil

class JointCommandPublisher(Node):
    def __init__(self, joint_name, command_file, recording_folder):
        super().__init__('joint_command_publisher')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.joint_name = joint_name
        self.command_file = command_file
        self.recording_folder = recording_folder
        self.publisher = self.create_publisher(JointState, '/joint_command', 10)
        self.sim_time = None
        self.current_joint_positions = {}  # To store the latest positions

        # Subscribe to the simulation clock
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        
        # Subscribe to the /joint_states topic to get the current joint position
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def clock_callback(self, msg):
        # Update the current simulation time in seconds
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def joint_state_callback(self, msg):
        # Update the current joint positions dictionary with the latest values
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def get_current_position(self):
        # Return the latest known position of the target joint
        return self.current_joint_positions.get(self.joint_name, 0.0)

    def wait_for_time(self, duration):
        # Wait until the simulation time advances by the specified duration
        if self.sim_time is None:
            self.get_logger().warn("Waiting for simulation time to be available...")
            while self.sim_time is None:
                rclpy.spin_once(self)
        start_time = self.sim_time
        while self.sim_time is None or (self.sim_time - start_time < duration):
            rclpy.spin_once(self)

    def send_joint_command_with_velocity(self, joint_name, end_pos):
        # Get the current position of the joint
        start_pos = self.get_current_position()
        print(f"Moving joint {joint_name} from {start_pos} to {end_pos}")

        # Create and publish the initial JointState message with velocity
        msg = JointState()
        msg.name = [joint_name]
        msg.position = [end_pos]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def execute_trajectory(self, joint_name):
        # Ensure the recording folder exists
        os.makedirs(self.recording_folder, exist_ok=True)

        # Read commands from the specified file
        with open(self.command_file, 'r') as file:
            lines = file.readlines()

        # Process each line in the command file
        for line in lines:
            L, R= map(float, line.strip().split())
            print(f"Executing command: Move from {L} to {R}")

            # Move to initial position L
            self.send_joint_command_with_velocity(self.joint_name, L)
            self.wait_for_time(3)  # Wait to reach initial position

            # Start ROS2 recording
            recording_filename = f"{joint_name}/bags/{L}_{R}".replace(" ", "_") + ".bag"            
            recording_path = os.path.join(self.recording_folder, recording_filename)
            if os.path.exists(recording_path):
                shutil.rmtree(recording_path)

            recording_process = subprocess.Popen(["ros2", "bag", "record", "-o", recording_path, 
                                                "/joint_states", "/joint_command", "/clock"])
            print(f"Recording started: {recording_filename}")

            # Short delay to ensure recording is ready
            self.wait_for_time(1)

            self.send_joint_command_with_velocity(self.joint_name, R)
            self.wait_for_time(3)


            # Stop recording after command is completed
            recording_process.terminate()
            print(f"Recording stopped: {recording_filename}")
            self.wait_for_time(0.5)

            self.send_joint_command_with_velocity(self.joint_name, 0.0) # Reset joint position to 0

def main(args=None):
    os.environ['ROS_DOMAIN_ID'] = '68'  # Set before initializing ROS
    rclpy.init(args=args)

    # Retrieve command file and recording folder from command-line arguments
    if len(sys.argv) < 3:
        print("Usage: python3 joint_command_publisher.py <command_file> <recording_folder>")
        return

    command_file = sys.argv[1]
    recording_folder = sys.argv[2]

    # Extract joint name from the command file name (assumes format '<joint_name>.txt')
    joint_name = os.path.splitext(os.path.basename(command_file))[0]

    joint_command_publisher = JointCommandPublisher(joint_name, command_file, recording_folder)
    joint_command_publisher.execute_trajectory(joint_name)

    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
