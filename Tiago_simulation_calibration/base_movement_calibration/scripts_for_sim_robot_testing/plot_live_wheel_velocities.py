#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock  # Import the Clock message type

VELOCITY_CHANGE_THRESHOLD = 50
# Define the wheel joint names
wheel_joints = [
    "wheel_front_left_joint",
    "wheel_front_right_joint",
    "wheel_rear_left_joint",
    "wheel_rear_right_joint"
]

# Node to handle ROS communication
class WheelVelocityPlotter(Node):
    def __init__(self):
        super().__init__('wheel_velocity_plotter')

        # Subscribe to /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )

        # Subscribe to /clock topic
        self.clock_subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

        self.wheel_velocities = {joint: [] for joint in wheel_joints}
        self.times = []
        self.current_time = None  # Placeholder for the simulation time

    def clock_callback(self, msg):
        # Extract time from the /clock topic
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def listener_callback(self, msg):
        if self.current_time is None:
            # Wait until we receive the first /clock message
            return

        # Use the simulation time from /clock instead of node clock
        self.times.append(self.current_time)

        for joint in wheel_joints:
            if joint in msg.name:
                index = msg.name.index(joint)
                velocity = msg.velocity[index]

                #Apply threshold filtering to ignore sudden large changes
                if len(self.wheel_velocities[joint]) > 0:
                    previous_velocity = self.wheel_velocities[joint][-1]
                    if abs(velocity - previous_velocity) > VELOCITY_CHANGE_THRESHOLD:
                        velocity = previous_velocity  # Ignore the spike

                self.wheel_velocities[joint].append(velocity)
            else:
                self.wheel_velocities[joint].append(0.0)


def live_plot():
    # Initialize the ROS node
    rclpy.init()
    plotter = WheelVelocityPlotter()

    # Initialize the plot
    fig, axs = plt.subplots(2, 2)
    ax_dict = {
        "wheel_front_left_joint": axs[0, 0],
        "wheel_front_right_joint": axs[0, 1],
        "wheel_rear_left_joint": axs[1, 0],
        "wheel_rear_right_joint": axs[1, 1]
    }

    # Set plot titles
    axs[0, 0].set_title('Front Left Wheel')
    axs[0, 1].set_title('Front Right Wheel')
    axs[1, 0].set_title('Rear Left Wheel')
    axs[1, 1].set_title('Rear Right Wheel')

    # Set axis labels
    for ax in axs.flat:
        ax.set(xlabel='Time (s)', ylabel='Velocity (m/s)')
        ax.grid()

    # Function to update the plot with new data
    def update_plot(frame):
        if len(plotter.times) == 0:
            return

        # Get the current time and define the 10-second window
        current_time = plotter.times[-1]
        time_window_start = current_time - 10.0

        # Find the index where the time window starts
        start_index = np.searchsorted(plotter.times, time_window_start)

        # Clear the previous plots
        for ax in ax_dict.values():
            ax.cla()

        # Plot the wheel velocities within the last 10 seconds
        for joint, ax in ax_dict.items():
            # Slice the data to only include the last 10 seconds
            times_to_plot = plotter.times[start_index:]
            velocities_to_plot = plotter.wheel_velocities[joint][start_index:]

            ax.plot(times_to_plot, velocities_to_plot, label=joint)
            ax.set_title(joint.replace("_", " ").title())
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Velocity (m/s)')
            ax.set_xlim([time_window_start, current_time])  # Limit the x-axis to the last 10 seconds
            ax.grid()
            ax.set_ylim(-25, 25)

    # Use FuncAnimation to continuously update the plot
    ani = FuncAnimation(fig, update_plot, interval=100)

    # Start the ROS 2 spinning and the matplotlib live plot
    def spin_node():
        while rclpy.ok():
            rclpy.spin_once(plotter, timeout_sec=0.1)

    import threading
    thread = threading.Thread(target=spin_node)
    thread.daemon = True
    thread.start()

    plt.tight_layout()
    plt.show()

    # Shutdown ROS after closing the plot
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    live_plot()
