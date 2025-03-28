import os
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


# Regex pattern to extract X, Y, Z, SECS values from the filename
pattern = r'x=(?P<X>-?\d+\.?\d*)y=(?P<Y>-?\d+\.?\d*)z=(?P<Z>-?\d+\.?\d*)secs=(?P<SECS>-?\d+\.?\d*)'

# Function to extract values from the filename for sorting
def extract_values_from_filename(filename):
    match = re.search(pattern, filename)
    if match:
        return (
            float(match.group('X')),
            float(match.group('Y')),
            float(match.group('Z')),
            float(match.group('SECS'))
        )
    else:
        return (float('inf'), float('inf'), float('inf'), float('inf'))  # Return high values if the pattern is not found


def plot_velocity_with_cutoff_and_avg(files, cutoff_s, cutoff_f, n_cols=3):
    n_files = len(files)
    n_rows = int(np.ceil(n_files / n_cols))  # Calculate the number of rows for the grid

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(15, n_rows * 3))  # Create a grid of subplots with smaller size
    axes = axes.flatten()  # Flatten the axes array to easily index

    for i, file_path in enumerate(files):
        data = pd.read_csv(file_path)
        time = data['Time']
        wheels = data.columns[1:]

        # Extract file name without extension for the title
        file_name = os.path.splitext(os.path.basename(file_path))[0]

        avg_velocities_per_wheel = []  # To store average velocity for each wheel

        for wheel in wheels:
            values_array = np.abs(data[wheel])

            adjusted_time = time - time.iloc[0]
            values_array = values_array[: len(adjusted_time)]

            # Apply the cutoff for time interval
            mask = (adjusted_time >= cutoff_s) & (adjusted_time <= cutoff_f)
            adjusted_time_cutoff = adjusted_time[mask]
            smoothed_values_cutoff = values_array[mask]

            # Calculate the average velocity in the interval
            avg_velocity = np.mean(smoothed_values_cutoff)
            avg_velocities_per_wheel.append(avg_velocity)  # Store the average velocity

            # Plot the velocity data within the cutoff interval for each wheel
            axes[i].plot(adjusted_time_cutoff, smoothed_values_cutoff, label=f'{wheel} | Avg Velocity: {avg_velocity:.6f} rad/s')

        # Calculate the overall average velocity across all four wheels
        overall_avg_velocity = np.mean(avg_velocities_per_wheel)

        # Customize plot for each file
        axes[i].set_xlabel('Time (s)')
        axes[i].set_ylabel('Velocity (rad/s)')
        axes[i].set_title(f'File: {file_name} |\n Avg Vel (all wheels): {overall_avg_velocity:.6f} rad/s')
        axes[i].set_ylim(0)
        axes[i].legend()

    # Remove unused axes if there are any
    for j in range(i+1, len(axes)):
        fig.delaxes(axes[j])

    # Adjust the layout
    plt.tight_layout()
    plt.show()

def plot_real_data(input_directory):

    # Collect all CSV file paths from "wheel_velocities" folders and sort them
    csv_files = []
    for root, dirs, files in os.walk(input_directory):
        for dir_name in dirs:
            if dir_name == "wheel_velocities" and "complex_random_trajectory" not in root:
                folder_path = os.path.join(root, dir_name)
                for filename in os.listdir(folder_path):
                    if filename.endswith(".csv"):
                        csv_files.append(os.path.join(folder_path, filename))

    # Sort the files based on extracted X, Y, Z, SECS values
    csv_files = sorted(csv_files, key=lambda f: extract_values_from_filename(os.path.basename(f)))


    # Cutoff interval (e.g., between 2.5 and 4 seconds)
    cutoff_s = 2.5
    cutoff_f = 4

    # Plot results with the cutoff interval and average velocity for all the files in a grid layout
    plot_velocity_with_cutoff_and_avg(csv_files, cutoff_s, cutoff_f, n_cols=3)