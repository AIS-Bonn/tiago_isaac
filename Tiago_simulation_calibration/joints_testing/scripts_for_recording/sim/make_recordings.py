import os
import sys
import subprocess

# Check if the required arguments are provided
if len(sys.argv) != 3:
    print("Usage: python3 run_commands.py <command_folder> <recording_folder>")
    sys.exit(1)

# Get the folder paths from the arguments
command_folder = sys.argv[1]
recording_folder = sys.argv[2]

# Check if the command folder exists
if not os.path.isdir(command_folder):
    print(f"Error: Command folder '{command_folder}' does not exist.")
    sys.exit(1)

# Check if the recording folder exists; create it if it doesn't
if not os.path.isdir(recording_folder):
    os.makedirs(recording_folder)
    print(f"Recording folder '{recording_folder}' created.")

# Iterate over all files in the command folder
for command_file in os.listdir(command_folder):
    command_path = os.path.join(command_folder, command_file)
    
    # Check if it is a file
    if os.path.isfile(command_path):
        print(f"Running command for: {command_file}")
        
        # Execute the script with the command file and recording folder as arguments
        result = subprocess.run(["python3", "record_simple_command.py", command_path, recording_folder])
        
        # Check if the execution was successful
        if result.returncode != 0:
            print(f"Error: Command failed for '{command_file}'")
        else:
            print(f"Successfully executed for '{command_file}'")

print("All commands executed.")
