import openvr
import time
import csv
import argparse
from datetime import datetime

def get_device_index(vr_system, device_class):
    """
    Get the index of the first device of the specified class.
    """
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        if vr_system.getTrackedDeviceClass(i) == device_class:
            return i
    return -1

def get_tracker_pose(vr_system, tracker_index):
    """
    Get the position and orientation of the tracker.
    """
    poses = vr_system.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)

    if poses[tracker_index].bPoseIsValid:
        pose = poses[tracker_index].mDeviceToAbsoluteTracking

        # Extract position
        tracker_position = [pose[0][3], pose[1][3], pose[2][3]]
        
        # Extract orientation (as a rotation matrix)
        tracker_orientation = [
            [pose[0][0], pose[0][1], pose[0][2]],
            [pose[1][0], pose[1][1], pose[1][2]],
            [pose[2][0], pose[2][1], pose[2][2]],
        ]

        return tracker_position, tracker_orientation
    else:
        return None, None

def main(name):
    # Initialize the OpenVR system
    vr_system = openvr.init(openvr.VRApplication_Other)

    # Get the index of the tracker
    tracker_index = get_device_index(vr_system, openvr.TrackedDeviceClass_GenericTracker)
    if tracker_index == -1:
        print("No tracker found.")
        return

    print(f"Tracker found with index: {tracker_index}")

    # Open a CSV file to write the data
    filename = f'{name}.csv'
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = ['timestamp', 'pos_x', 'pos_y', 'pos_z', 'rot_00', 'rot_01', 'rot_02', 'rot_10', 'rot_11', 'rot_12', 'rot_20', 'rot_21', 'rot_22']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        try:
            while True:
                # Get the tracker's pose
                position, orientation = get_tracker_pose(vr_system, tracker_index)

                if position and orientation:
                    print(f"Tracker Position: {position}")
                    print(f"Tracker Orientation: {orientation}")
                    # Get the current system time as a timestamp
                    # timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                    timestamp = int(time.time() * 1000)


                    # Write the data to the CSV file
                    writer.writerow({
                        'timestamp': timestamp,
                        'pos_x': position[0],
                        'pos_y': position[1],
                        'pos_z': position[2],
                        'rot_00': orientation[0][0],
                        'rot_01': orientation[0][1],
                        'rot_02': orientation[0][2],
                        'rot_10': orientation[1][0],
                        'rot_11': orientation[1][1],
                        'rot_12': orientation[1][2],
                        'rot_20': orientation[2][0],
                        'rot_21': orientation[2][1],
                        'rot_22': orientation[2][2]
                    })

                    print(f"Recorded tracker data at {timestamp}")

                else:
                    print("Tracker pose is not valid.")

                time.sleep(1/60)  # Adjust the sleep time as needed

        except KeyboardInterrupt:
            print("Tracking stopped.")

    # Shutdown the OpenVR system
    openvr.shutdown()

if __name__ == "__main__":
    # Argument parser setup
    parser = argparse.ArgumentParser(description='Track VR tracker and record data.')
    parser.add_argument('name', type=str, help='Name to use for the CSV file')

    args = parser.parse_args()

    # Run the main function with the provided name argument
    main(args.name)