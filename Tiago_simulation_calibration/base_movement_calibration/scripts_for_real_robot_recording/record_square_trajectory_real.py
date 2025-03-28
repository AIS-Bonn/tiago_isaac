#!/usr/bin/env python
"""
This script controls a robot in a ROS (Robot Operating System) environment by publishing velocity commands to a specific topic
and recording sensor data during the robot's movement. The script takes two arguments: speed (for the linear velocities)
and duration (in seconds) for which to execute each movement (forward, left, backward, right).

Usage: python script.py <SPEED> <SECS>
    - SPEED: Linear velocity magnitude for all movements (float)
    - SECS: Duration for which to execute each movement (float)
"""

import os
import sys
import time
import rospy
import subprocess
import shutil
import signal
from geometry_msgs.msg import Twist

def main(speed, secs):

    # Create recordings directory if it doesn't exist
    try:
        os.makedirs("recordings/")
    except OSError:
        pass  # Directory already exists

    bag_filename = "square_trajectory_speed={}secs={}.bag".format(speed, secs)
    bag_filepath = os.path.join("recordings/", bag_filename)

    # Remove existing bag file if it exists
    if os.path.exists(bag_filepath):
        os.remove(bag_filepath)

    record_cmd = [
        'rosbag', 'record', '-O', bag_filepath,
        '/joint_states', '/mobile_base_controller/cmd_vel', '/mobile_base_controller/odom'
    ]

    # Initialize ROS node
    rospy.init_node('square_trajectory_publisher')

    # Start recording process
    print("Starting recording process")
    record_process = subprocess.Popen(
        record_cmd,
        stdout=open(os.devnull, 'wb'),
        stderr=open(os.devnull, 'wb'),
        preexec_fn=os.setsid
    )

    # Create publisher for '/mobile_base_controller/cmd_vel'
    cmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    # Give publisher some time to connect
    time.sleep(2)

    rate = rospy.Rate(100)  # 100 Hz

    def publish_command(x, y, z, duration):
        command_msg = Twist()
        command_msg.linear.x = x
        command_msg.linear.y = y
        command_msg.linear.z = 0.0
        command_msg.angular.x = 0.0
        command_msg.angular.y = 0.0
        command_msg.angular.z = z

        start_time = time.time()
        print("Started publishing command at 100 Hz")

        while time.time() - start_time < duration and not rospy.is_shutdown():
            cmd_pub.publish(command_msg)
            rate.sleep()
        print("Finished publishing command")

    try:
        # --- First: Move forward ---
        print("Moving forward")
        publish_command(x=speed, y=0, z=0, duration=secs)

        # --- Second: Move left ---
        print("Moving left")
        publish_command(x=0, y=-speed, z=0, duration=secs)

        # --- Third: Move backward ---
        print("Moving backward")
        publish_command(x=-speed, y=0, z=0, duration=secs)

        # --- Fourth: Move right ---
        print("Moving right")
        publish_command(x=0, y=speed, z=0, duration=secs)

    finally:
        # Stop publishing commands
        print("Stopped publishing commands")

        # Send stop command
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        cmd_pub.publish(stop_msg)
        print("Published stop command")

        # Stop the recording process
        os.killpg(os.getpgid(record_process.pid), signal.SIGINT)
        record_process.wait()
        print("Stopped recording process")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python script.py <SPEED> <SECS>")
        sys.exit(1)
    SPEED = float(sys.argv[1])
    SECS = float(sys.argv[2])
    main(SPEED, SECS)
