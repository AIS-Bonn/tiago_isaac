import time
import subprocess
import sys

# Define the function to execute the command
def execute_command(x, y, z, secs):
    command = ["python3", "record_simple_trajectory_real.py", str(x), str(y), str(z), str(secs)]
    print(f"Executing: {' '.join(command)}")
    subprocess.run(command)

    time.sleep(secs + 3)


    # # Execute the reset command to move the robot back
    # reset_command = ["python3", "record_simple_trajectory_real.py", str(-x), str(-y), str(-z), str(secs)]
    # print(f"Executing reset: {' '.join(reset_command)}")
    # subprocess.run(reset_command)
    # # Wait  before executing the next command
    # time.sleep(secs + 2)

# Function to read the file and execute commands
def process_file(file_path):
    with open(file_path, "r") as file:
        for line in file:
            # Skip empty lines and lines starting with #
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            
            # Parse the line to get X, Y, Z, SECS
            try:
                x, y, z, secs = map(float, line.split())
                execute_command(x, y, z, secs)
            except ValueError:
                print(f"Invalid line format: {line}")

# Main function to run the script
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py command_file")
        sys.exit(1)
    commands_file = sys.argv[1]
    process_file(commands_file)