import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tkinter as tk
import threading
import os
import yaml
# List of joints
joint_names = [
    'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint',
    'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint', 'arm_right_1_joint',
    'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint',
    'arm_right_6_joint', 'arm_right_7_joint', 'gripper_left_left_finger_joint',
    'gripper_left_right_finger_joint', 'gripper_right_left_finger_joint',
    'gripper_right_right_finger_joint', 'head_1_joint', 'head_2_joint', 'torso_lift_joint'
]

# Define min and max values for each joint
joint_limits = {
    'arm_left_1_joint': (-1.1780972450961724, 1.5707963267948966),
    'arm_left_2_joint': (-1.1780972450961724, 1.5707963267948966),
    # 'arm_left_2_joint': (-1.1780972450961724, 0.75),#1.5707963267948966), #Artificial limit so it does not touch the floor
    'arm_left_3_joint': (-0.7853981633974483, 3.9269908169872414),
    'arm_left_4_joint': (-0.39269908169872414, 2.356194490192345),
    'arm_left_5_joint': (-2.0943951023931953, 2.0943951023931953),
    'arm_left_6_joint': (-1.413716694115407, 1.413716694115407),
    'arm_left_7_joint': (-2.0943951023931953, 2.0943951023931953),
    'arm_right_1_joint': (-1.1780972450961724, 1.5707963267948966),
    'arm_right_2_joint': (-1.1780972450961724, 1.5707963267948966),
    # 'arm_right_2_joint': (-1.1780972450961724, 0.75),# 1.5707963267948966), #Artificial limit so it does not touch the floor
    'arm_right_3_joint': (-0.7853981633974483, 3.9269908169872414),
    'arm_right_4_joint': (-0.39269908169872414, 2.356194490192345),
    'arm_right_5_joint': (-2.0943951023931953, 2.0943951023931953),
    'arm_right_6_joint': (-1.413716694115407, 1.413716694115407),
    'arm_right_7_joint': (-2.0943951023931953, 2.0943951023931953),
    'gripper_left_left_finger_joint': (0.0, 0.045),
    'gripper_left_right_finger_joint': (0.0, 0.045),
    'gripper_right_left_finger_joint': (0.0, 0.045),
    'gripper_right_right_finger_joint': (0.0, 0.045),
    'head_1_joint': (-1.3089969389957472, 1.3089969389957472),
    'head_2_joint': (-1.0471975511965976, 0.7853981633974483),
    'torso_lift_joint': (0.0, 0.35)
}

class JointControlNode(Node):
    def __init__(self):
        super().__init__('joint_control_gui')
        self.publisher = self.create_publisher(JointState, '/joint_command', 10)
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.current_joint_positions = {joint: 0.0 for joint in joint_names}
        self.gui = None  # Will be set later by the GUI

    def send_joint_command(self, joint_names, positions):
        msg = JointState()
        msg.name = joint_names
        msg.position = positions
        self.publisher.publish(msg)

    def joint_state_callback(self, msg):
        # Update current_joint_positions
        for name, position in zip(msg.name, msg.position):
            if name in self.current_joint_positions:
                self.current_joint_positions[name] = position
        # Schedule GUI update
        if self.gui:
            # Make a copy to avoid threading issues
            positions_copy = self.current_joint_positions.copy()
            self.gui.window.after(0, self.gui.update_state_sliders, positions_copy)


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
class JointControlGUI:
    def __init__(self, node):
        self.node = node
        self.node.gui = self  # Link back to the GUI
        self.window = tk.Tk()
        self.window.title("Joint Control GUI")
        
        # Directory for storing poses
        self.poses_dir = "../poses"
        os.makedirs(self.poses_dir, exist_ok=True)

        # Dictionary to store state and command slider variables
        self.state_sliders = {}
        self.command_sliders = {}

        # Main layout frame
        main_frame = tk.Frame(self.window)
        main_frame.pack()

        # Column labels
        label_frame = tk.Frame(main_frame)
        label_frame.grid(row=0, column=0, columnspan=3)
        tk.Label(label_frame, text="Current joint angle").grid(row=0, column=0, padx=10)
        tk.Label(label_frame, text="Joint").grid(row=0, column=1, padx=10)
        tk.Label(label_frame, text="Target joint angle").grid(row=0, column=2, padx=10)

        # Create sliders for each joint
        for i, joint_name in enumerate(joint_names):
            min_val, max_val = joint_limits[joint_name]
            self.create_joint_sliders(main_frame, joint_name, min_val, max_val, i + 1)

        # Buttons column
        button_frame = tk.Frame(main_frame)
        button_frame.grid(row=1, column=3, rowspan=len(joint_names), padx=10, pady=10, sticky=tk.N)

        # Add buttons to the button column
        reset_button = tk.Button(button_frame, text="Reset Target Angles", command=self.reset_target_angles)
        reset_button.pack(pady=5)

        save_pose_button = tk.Button(button_frame, text="Save Pose", command=self.save_pose)
        save_pose_button.pack(pady=5)

        self.pose_frame = tk.Frame(button_frame)  # Frame for dynamically created pose buttons
        self.pose_frame.pack(pady=10)

        # Load initial poses
        self.load_pose_buttons()

        # Start periodic updates every 100ms
        self.update_gui()

    def create_joint_sliders(self, parent, joint_name, min_val, max_val, row):
        # State slider (left) - reflects the current joint state
        state_var = tk.DoubleVar()
        state_slider = tk.Scale(
            parent, variable=state_var, from_=min_val, to=max_val, resolution=0.0001,
            orient=tk.HORIZONTAL, length=150, state=tk.DISABLED, width=10
        )
        state_slider.grid(row=row, column=0, padx=5, pady=2)

        # Joint label (center)
        label = tk.Label(parent, text=joint_name)
        label.grid(row=row, column=1, padx=5, pady=2)

        # Command slider (right) - user-controlled, sends joint commands
        command_var = tk.DoubleVar()
        command_slider = tk.Scale(
            parent, variable=command_var, from_=min_val, to=max_val, resolution=0.0001,
            orient=tk.HORIZONTAL, length=150, width=10,
            command=lambda val, j=joint_name: self.send_command(j, float(val))
        )
        command_slider.grid(row=row, column=2, padx=5, pady=2)

        # Store sliders in dictionaries
        self.state_sliders[joint_name] = state_var
        self.command_sliders[joint_name] = command_var

    def send_command(self, joint_name, position):
        self.node.send_joint_command([joint_name], [position])

    def update_state_sliders(self, joint_positions):
        for joint_name, position in joint_positions.items():
            if joint_name in self.state_sliders:
                # Update state slider value safely
                self.state_sliders[joint_name].set(position)

    def reset_target_angles(self):
        # Set all target angles to 0 and send as a single command
        joint_names = list(self.command_sliders.keys())
        positions = [0.0] * len(joint_names)
        
        # Update all sliders to show the reset values
        for command_var in self.command_sliders.values():
            command_var.set(0.0)
        
        # Send all reset commands in a single JointState message
        self.node.send_joint_command(joint_names, positions)
        print("All target joint angles reset to 0")

    def save_pose(self):
        pose_id = len(os.listdir(self.poses_dir))
        pose_path = os.path.join(self.poses_dir, f"pose{pose_id}.txt")
        pose_data = {joint: var.get() for joint, var in self.command_sliders.items()}
        save_pose(pose_path, pose_data)
        print(f"Pose saved to {pose_path}.")
        self.load_pose_buttons()

    def load_pose(self, pose_path):
        pose_data = read_pose(pose_path)
        if pose_data:
            joints = []
            positions = []
            for joint, position in pose_data.items():
                if joint in self.command_sliders:
                    self.command_sliders[joint].set(position)
                    joints.append(joint)
                    positions.append(position)
            
            self.node.send_joint_command(joints, positions)
            print(f"Loaded pose from {pose_path}.")


    def load_pose_buttons(self):
        for widget in self.pose_frame.winfo_children():
            widget.destroy()

        pose_files = [f for f in os.listdir(self.poses_dir) if f.endswith(".txt")]
        # sort alphabetically
        pose_files.sort()
        for pose_file in pose_files:
            pose_path = os.path.join(self.poses_dir, pose_file)
            button = tk.Button(self.pose_frame, text=f"Load {pose_file}",
                               command=lambda path=pose_path: self.load_pose(path))
            button.pack(pady=2)

    def update_gui(self):
        # Call update_state_sliders with the latest positions from the node
        joint_positions = self.node.current_joint_positions.copy()
        self.update_state_sliders(joint_positions)
        
        # Schedule the next GUI update
        self.window.after(100, self.update_gui)





def main():
    rclpy.init()
    node = JointControlNode()
    gui = JointControlGUI(node)
    
    # Start rclpy.spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        gui.window.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()