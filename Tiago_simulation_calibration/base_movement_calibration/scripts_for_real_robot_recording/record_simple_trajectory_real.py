#!/usr/bin/env python
"""
This script controls a robot in a ROS (Robot Operating System) environment by publishing velocity commands to a specific topic
and recording sensor data during the robot's movement. The script takes four arguments: linear velocity in the x and y directions,
angular velocity around the z-axis, and the duration (in seconds) for which to execute the command.

Usage: python script.py <X> <Y> <Z> <secs>
    - X: Linear velocity in the x direction (float)
    - Y: Linear velocity in the y direction (float)
    - Z: Angular velocity around the z-axis (float)
    - secs: Duration for which to execute the movement (float)
"""

import os
import sys
import time
import rospy
import subprocess
import shutil
import signal
from geometry_msgs.msg import Twist

def main(x, y, z, secs):

    inverse_command_dif_to_forward_command = 0.0

    if secs <= inverse_command_dif_to_forward_command:
        rospy.logwarn("Duration 'secs' should be greater than 1 for the inverse command to run.")
        inverse_duration = 0
    else:
        inverse_duration = secs - inverse_command_dif_to_forward_command


    # Create recordings directory if it doesn't exist
    try:
        os.makedirs("recordings/")
    except OSError:
        pass  # Directory already exists

    bag_filename = "x={}y={}z={}secs={}.bag".format(x, y, z, secs)
    bag_filepath = os.path.join("recordings/", bag_filename)

    # Remove existing bag file if it exists
    if os.path.exists(bag_filepath):
        os.remove(bag_filepath)

    record_cmd = [
        'rosbag', 'record', '-O', bag_filepath,
        '/joint_states', '/mobile_base_controller/cmd_vel', '/mobile_base_controller/odom'
    ]

    # Initialize ROS node
    rospy.init_node('command_publisher')


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

    # --- First command: Publish initial command for 'secs' seconds ---
    command_msg = Twist()
    command_msg.linear.x = x
    command_msg.linear.y = y
    command_msg.linear.z = 0.0
    command_msg.angular.x = 0.0
    command_msg.angular.y = 0.0
    command_msg.angular.z = z

    start_time = time.time()
    print("start_time: {}".format(start_time))
    print("Started publishing initial command at 100 Hz")

    try:
        # Publish initial command for 'secs' seconds
        while time.time() - start_time < secs and not rospy.is_shutdown():
            cmd_pub.publish(command_msg)
            print("Published initial command")
            rate.sleep()

        # # --- Second command: Publish inverse command for 'secs - 2' seconds ---
        # inverse_command_msg = Twist()
        # inverse_command_msg.linear.x = -x
        # inverse_command_msg.linear.y = -y
        # inverse_command_msg.linear.z = 0.0
        # inverse_command_msg.angular.x = 0.0
        # inverse_command_msg.angular.y = 0.0
        # inverse_command_msg.angular.z = -z

        # inverse_start_time = time.time()
        # inverse_duration = secs - inverse_command_dif_to_forward_command

        # print("Started publishing inverse command at 100 Hz")

        # while time.time() - inverse_start_time < inverse_duration and not rospy.is_shutdown():
        #     cmd_pub.publish(inverse_command_msg)
        #     print("Published inverse command")
        #     rate.sleep()
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
    if len(sys.argv) != 5:
        print("Usage: python script.py <X> <Y> <Z> <secs>")
        sys.exit(1)
    X = float(sys.argv[1])
    Y = float(sys.argv[2])
    Z = float(sys.argv[3])
    secs = float(sys.argv[4])
    main(X, Y, Z, secs)
