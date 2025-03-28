import os

# List of different <CMD> strings to substitute into the command
cmd_list = ["squares", "forward_circle", "sideward_circle"]#["x", "y", "z", "xy", "r_forwards", "r_sidewards", "x0.5y"]

# Base command template
command_template = "python3 extract_full_pose_recordings_to_csv.py recordings/tracker_session_23_10_{cmd}.csv recordings/{cmd}/bags/ recordings/{cmd}/poses/"

# Loop over each command and execute it
for cmd in cmd_list:
    # Substitute the <CMD> placeholder with the actual command
    full_command = command_template.format(cmd=cmd)
    
    # Execute the command
    print(f"Executing: {full_command}")
    os.system(full_command)
