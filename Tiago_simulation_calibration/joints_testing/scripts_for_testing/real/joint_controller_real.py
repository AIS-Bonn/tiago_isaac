import paramiko
import os
import yaml
import threading
import time
import tkinter as tk
from tkinter import Button, Label, Frame


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

def read_pose(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Warning: File {file_path} not found.")
        return None
    except Exception as e:
        print(f"Error reading file {file_path}: {e}")
        return None

def save_pose(file_path, joint_states):
    try:
        with open(file_path, 'w') as file:
            yaml.dump(joint_states, file, default_flow_style=False)
        print(f"Pose saved to {file_path}.")
    except Exception as e:
        print(f"Error saving pose to {file_path}: {e}")

class RemoteROSController:
    def __init__(self, hostB, userB, passwordB, hostC, userC, passwordC):
        self.hostB = hostB
        self.userB = userB
        self.passwordB = passwordB
        self.hostC = hostC
        self.userC = userC
        self.passwordC = passwordC

        # Locks for thread safety
        self.ssh_lock = threading.Lock()
        self.sftp_lock = threading.Lock()

        # Establish SSH connection to B
        self.ssh_B = paramiko.SSHClient()
        self.ssh_B.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh_B.connect(hostB, username=userB, password=passwordB)

        # Create a tunnel from B to C
        transport = self.ssh_B.get_transport()
        dest_addr = (hostC, 22)
        local_addr = ('localhost', 0)
        self.tunnel = transport.open_channel("direct-tcpip", dest_addr, local_addr)

        print(f"Tunnel created from {hostB} to {hostC}")

        # Connect to C via the tunnel
        self.ssh_C = paramiko.SSHClient()
        self.ssh_C.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh_C.connect('localhost', port=self.tunnel.getpeername()[1],
                           username=userC, password=passwordC, sock=self.tunnel)

        print(f"Connected to {hostC} via tunnel")

        # Open SFTP session
        self.sftp = self.ssh_C.open_sftp()

    def read_remote_file(self, filepath):
        with self.sftp_lock:
            try:
                with self.sftp.file(filepath, 'r') as f:
                    return yaml.safe_load(f)
            except FileNotFoundError:
                print(f"File {filepath} not found on remote host.")
                return None
            except Exception as e:
                print(f"Error reading file {filepath} on remote host: {e}")
                return None

    def write_remote_file(self, filepath, content):
        with self.sftp_lock:
            try:
                with self.sftp.file(filepath, 'w') as f:
                    f.write(content)
                print(f"File {filepath} written successfully.")
            except Exception as e:
                print(f"Error writing file {filepath} on remote host: {e}")

    def close_connections(self):
        with self.ssh_lock, self.sftp_lock:
            self.sftp.close()
            self.ssh_C.close()
            self.ssh_B.close()
            print("SSH connections closed.")

class PoseManager:
    def __init__(self, gui):
        self.gui = gui
        self.poses_dir = "../poses"
        os.makedirs(self.poses_dir, exist_ok=True)
        self.load_pose_buttons()

    def load_pose_buttons(self):
        for widget in self.gui.pose_frame.winfo_children():
            widget.destroy()

        pose_files = [f for f in os.listdir(self.poses_dir) if f.startswith("pose") and f.endswith(".txt")]
        for pose_file in sorted(pose_files):
            pose_path = os.path.join(self.poses_dir, pose_file)
            self.create_pose_button(pose_file, pose_path)

    def create_pose_button(self, pose_name, pose_path):
        button = Button(self.gui.pose_frame, text=f"Load {pose_name}",
                        command=lambda: self.gui.load_pose(pose_path))
        button.pack(pady=2)

class JointControlGUI:
    def __init__(self, remote_ros_controller):
        self.remote_ros = remote_ros_controller
        self.window = tk.Tk()
        self.window.title("Joint Control GUI")

        # Dictionary to store state and command slider variables
        self.state_sliders = {}
        self.command_sliders = {}

        # Dictionary to store desired positions
        self.desired_positions = {}

        # Column labels
        label_frame = Frame(self.window)
        label_frame.pack()
        Label(label_frame, text="Current joint angle").pack(side=tk.LEFT, padx=75)
        Label(label_frame, text="Target joint angle").pack(side=tk.RIGHT, padx=75)

        # Create sliders for each joint
        for joint_name in joint_limits.keys():
            min_val, max_val = joint_limits.get(joint_name, (-1.0, 1.0))  # Default if no limit found
            self.create_joint_sliders(joint_name, min_val, max_val)
            self.desired_positions[joint_name] = 0.0

        # Reset button
        reset_button = Button(self.window, text="0-Pose", command=self.zero_pose)
        reset_button.pack(pady=10)

        # Save Pose button
        save_button = Button(self.window, text="Save Pose", command=self.save_pose)
        save_button.pack(pady=5)

        # Load Pose buttons frame
        self.pose_frame = Frame(self.window)
        self.pose_frame.pack(pady=10)

        # Initialize pose manager
        self.pose_manager = PoseManager(self)

        # Load initial current joint angles
        self.initialize_joint_sliders()

        # Start periodic updates
        self.update_current_joint_sliders()
        self.window.after(1500, self.write_joint_states_to_file)

    def initialize_joint_sliders(self):
        current_joint_angles = self.remote_ros.read_remote_file('/home/pal/current_joint_angles.txt')
        if current_joint_angles:
            for joint_name, angle in current_joint_angles.items():
                if joint_name in self.state_sliders:
                    self.state_sliders[joint_name].set(angle)
                    self.command_sliders[joint_name].set(angle)
                    self.desired_positions[joint_name] = angle

    def create_joint_sliders(self, joint_name, min_val, max_val):
        frame = Frame(self.window)
        frame.pack()

        label = Label(frame, text=joint_name)
        label.pack(side=tk.LEFT)

        state_var = tk.DoubleVar()
        state_slider = tk.Scale(
            frame, variable=state_var, from_=min_val, to=max_val, resolution=0.0001,
            orient=tk.HORIZONTAL,
            sliderlength=50,
            width=50,
            length=350, state=tk.DISABLED
        )
        state_slider.pack(side=tk.LEFT)

        command_var = tk.DoubleVar()
        command_slider = tk.Scale(
            frame, variable=command_var, from_=min_val, to=max_val, resolution=0.0001,
            orient=tk.HORIZONTAL,
            sliderlength=50,
            width=50,
            length=350,
            command=lambda val, j=joint_name: self.update_desired_position(j, float(val))
        )
        command_slider.pack(side=tk.RIGHT)

        self.state_sliders[joint_name] = state_var
        self.command_sliders[joint_name] = command_var

    def update_desired_position(self, joint_name, position):
        self.desired_positions[joint_name] = position

    def write_joint_states_to_file(self):
        joint_states = {joint: pos for joint, pos in self.desired_positions.items()}
        content = yaml.dump(joint_states, default_flow_style=False)
        self.remote_ros.write_remote_file('/home/pal/desired_joint_states.txt', content)
        self.window.after(1500, self.write_joint_states_to_file)

    def update_current_joint_sliders(self):
        current_joint_angles = self.remote_ros.read_remote_file('/home/pal/current_joint_angles.txt')
        if current_joint_angles:
            for joint_name, angle in current_joint_angles.items():
                if joint_name in self.state_sliders:
                    self.state_sliders[joint_name].set(angle)
        self.window.after(500, self.update_current_joint_sliders)

    def zero_pose(self):
        for joint_name, command_var in self.command_sliders.items():
            current_angle = 0.0
            command_var.set(current_angle)
            self.desired_positions[joint_name] = current_angle

    def save_pose(self):
        pose_id = len(os.listdir(self.pose_manager.poses_dir))
        file_name = f"pose{pose_id}.txt"
        file_path = os.path.join(self.pose_manager.poses_dir, file_name)
        save_pose(file_path, self.desired_positions)
        self.pose_manager.load_pose_buttons()

    def load_pose(self, pose_path):
        joint_states = read_pose(pose_path)
        if joint_states:
            for joint_name, angle in joint_states.items():
                if joint_name in self.command_sliders:
                    self.command_sliders[joint_name].set(angle)
                    self.desired_positions[joint_name] = angle
            print(f"Loaded pose from {pose_path}.")

    def run(self):
        self.window.mainloop()

def main():
    hostB = '10.7.3.128'
    userB = 'nimbro_home'
    passwordB = 'RoboCup@Home'

    hostC = 'tiago'
    userC = 'pal'
    passwordC = 'pal'

    remote_ros_controller = RemoteROSController(hostB, userB, passwordB, hostC, userC, passwordC)

    try:
        gui = JointControlGUI(remote_ros_controller)
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        remote_ros_controller.close_connections()

if __name__ == '__main__':
    main()
