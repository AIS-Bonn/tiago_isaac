import time
import subprocess
import sys

# Define the function to execute the command
def execute_command(speed, secs):
    command = ["python3", "record_square_trajectory_real.py", str(speed), str(secs)]
    print(f"Executing: {' '.join(command)}")
    subprocess.run(command)

    # Wait for the command to finish and give some buffer time before the next one
    time.sleep(secs + 3)

# Function to read the file and execute commands (in a square trajectory)
def process_file(file_path):
    with open(file_path, "r") as file:
        for line in file:
            # Skip empty lines and lines starting with #
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            # Parse the line to get SPEED and SECS
            try:
                speed, secs = map(float, line.split())
                # Execute the square trajectory pattern
                print("Starting square trajectory with speed {} for {} seconds each segment.".format(speed, secs))

                execute_command(speed, secs)

                print("Completed square trajectory.\n")
            except ValueError:
                print(f"Invalid line format: {line}")

# Main function to run the script
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py command_file")
        sys.exit(1)
    commands_file = sys.argv[1]
    process_file(commands_file)
