import time
import os
import yaml
import paramiko
import threading

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

class PeriodicPoseSwitcher:
    def __init__(self, remote_ros_controller, poses_dir):
        self.remote_ros = remote_ros_controller
        self.poses_dir = poses_dir
        self.pose_files = ["pose0.txt", "pose1.txt"]
        self.pose_index = 0
        self.rotation_directions = [60.0, -60.0]  # Degrees per second
        self.rotation_index = 0

    def load_pose(self, pose_path):
        try:
            with open(pose_path, 'r') as file:
                return yaml.safe_load(file)
        except FileNotFoundError:
            print(f"File {pose_path} not found.")
            return None
        except Exception as e:
            print(f"Error reading file {pose_path}: {e}")
            return None

    def switch_pose(self):
        while True:
            # Load the current pose
            pose_file = self.pose_files[self.pose_index]
            pose_path = os.path.join(self.poses_dir, pose_file)
            joint_states = self.load_pose(pose_path)

            if joint_states:
                # Convert the joint states to YAML format
                pose_content = yaml.dump(joint_states, default_flow_style=False)
                # Write the pose to the remote file on C
                self.remote_ros.write_remote_file('/home/pal/desired_joint_states.txt', pose_content)

            # Write the twist command to the remote file on C
            twist_command = {
                'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': self.rotation_directions[self.rotation_index] * (3.14159 / 180)}
            }
            print(twist_command)
            twist_content = yaml.dump(twist_command, default_flow_style=False)
            self.remote_ros.write_remote_file('/home/pal/desired_base_movement.txt', twist_content)

            # Switch to the next pose and rotation direction
            self.pose_index = (self.pose_index + 1) % len(self.pose_files)
            self.rotation_index = (self.rotation_index + 1) % len(self.rotation_directions)

            # Wait for 2 seconds before switching again
            time.sleep(2)

if __name__ == '__main__':
    hostB = '10.7.3.128'
    userB = 'nimbro_home'
    passwordB = 'RoboCup@Home'

    hostC = 'tiago'
    userC = 'pal'
    passwordC = 'pal'

    poses_dir = "../poses"

    remote_ros_controller = RemoteROSController(hostB, userB, passwordB, hostC, userC, passwordC)
    pose_switcher = PeriodicPoseSwitcher(remote_ros_controller, poses_dir)

    try:
        pose_switcher.switch_pose()
    except KeyboardInterrupt:
        pass
    finally:
        remote_ros_controller.close_connections()
