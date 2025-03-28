import os, sys
import subprocess

def batch_process(simulation_base_dir):
    # Iterate over all folders in the simulation directory
    for command_folder in os.listdir(simulation_base_dir):
        command_folder_path = os.path.join(simulation_base_dir, command_folder)
        
        if os.path.isdir(command_folder_path):  # Check if it's a directory
            # Iterate over the number folders inside each command folder
            for number_folder in os.listdir(command_folder_path):
                number_folder_path = os.path.join(command_folder_path, number_folder)
                
                if os.path.isdir(number_folder_path):  # Check if it's a directory
                    # Check for the "bags" folder inside each number folder
                    bags_folder_path = os.path.join(number_folder_path, 'bags')
                    
                    if os.path.isdir(bags_folder_path):  # Ensure the "bags" folder exists
                        # Define the output folder path for "wheel_velocities"
                        wheel_velocities_folder_path = os.path.join(number_folder_path, 'wheel_velocities')
                        
                        # Create the "wheel_velocities" folder if it doesn't exist
                        if not os.path.exists(wheel_velocities_folder_path):
                            os.makedirs(wheel_velocities_folder_path)
                        
                        # Construct the command to run the script
                        script_path = 'extract_wheel_data_from_ros2bag.py'  # Assuming the script is in the same directory as this script
                        command = [
                            'python3',
                            script_path,
                            bags_folder_path,
                            wheel_velocities_folder_path
                        ]
                        

                        # Run the command using subprocess
                        subprocess.run(command, check=True)



                        poses_folder_path = os.path.join(number_folder_path, 'poses')

                        # Create the "wheel_velocities" folder if it doesn't exist
                        if not os.path.exists(poses_folder_path):
                            os.makedirs(poses_folder_path)

                        # Construct the command to run the script
                        script_path = 'extract_odom_poses_to_csv.py'  # Assuming the script is in the same directory as this script
                        command = [
                            'python3',
                            script_path,
                            bags_folder_path,
                            poses_folder_path
                        ]
                        
                        # Run the command using subprocess
                        subprocess.run(command, check=True)

    print("Script execution for all bags folders completed.")


if __name__ == "__main__":
    # Get folder paths from arguments
    if len(sys.argv) != 2:
        print("Usage: python script.py <simulation dir>")
    else:
        simulation_base_dir = sys.argv[1]
        
        batch_process(simulation_base_dir)
