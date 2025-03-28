This folder contains recorded data and scripts to process this data.


Workflow/Process of recording and processing simple recorded command data for the real robot:
1. put scripts_for_real_robot_recording folder on robot
2. If trajectory tracking: Put the tracking device on the robot, and run tracker_record.py while doing one recording session.
3. run automated_recording_real.py
4. move recorded data into trajectory_data/real
5. use extract_pose_recordings_to_csv.py to extract the poses from the .csv file created by tracker_record.py and  use extract_wheel_velocities_from_rosbag.py to extract the wheel velocities from the bag files


