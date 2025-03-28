import os
import shutil
import sys

def organize_files(main_folder, new_folder_name):
    # Loop through each subfolder in the main folder
    for subdir in os.listdir(main_folder):
        subdir_path = os.path.join(main_folder, subdir)
        
        # Ensure we only work with directories
        if os.path.isdir(subdir_path):
            new_folder_path = os.path.join(subdir_path, new_folder_name)
            
            # Create the new folder inside the subfolder
            os.makedirs(new_folder_path, exist_ok=True)
            
            # Loop through all files in the subfolder
            for file_name in os.listdir(subdir_path):
                file_path = os.path.join(subdir_path, file_name)
                
                if file_name == "bags":
                    shutil.move(file_path, os.path.join(new_folder_path, file_name))

if __name__ == "__main__":
    # Get folder paths from arguments
    if len(sys.argv) != 3:
        print("Usage: python script.py <main_folder> <new_folder_name>")
    else:
        main_folder = sys.argv[1]
        new_folder_name = sys.argv[2]
        
        organize_files(main_folder, new_folder_name)
