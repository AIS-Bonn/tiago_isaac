import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


# Function to remove outliers based on z-score
def remove_outliers(df, column):
    df = df[np.abs(df[column]) <= 20]
    return df

# Function to compute a smoothed average using a moving window
def smooth_data(data, window_size=10):
    return data.rolling(window=window_size, center=True).mean()

# Function to plot wheel velocities from multiple simulation files and a real data CSV file
def plot_wheel_velocities(simulation_files, real_file, cutoff, x_value, y_value, z_value, secs):
    # Read the real CSV file into a DataFrame
    df_real = pd.read_csv(real_file)
    df_real['Time'] -= df_real['Time'].min()
    df_real = df_real.dropna()
    df_real = df_real[df_real['Time'] <= cutoff]

    # Create a figure with 4 subplots
    fig, axs = plt.subplots(4, 1, figsize=(10, 12))

    wheel_positions = ['Front Left', 'Front Right', 'Rear Left', 'Rear Right']
    colors = ['blue', 'green', 'orange', 'purple', 'brown']  # Different colors for different timesteps

    # Loop through the simulation files
    for idx, simulation_file in enumerate(simulation_files):
        timestep_label = os.path.basename(os.path.dirname(os.path.dirname(simulation_file)))  # Extract the timestep from the path
        
        # Read the simulation CSV file into a DataFrame
        df_sim = pd.read_csv(simulation_file)
        df_sim['Time'] -= df_sim['Time'].min()
        df_sim = df_sim.dropna()
        df_sim = df_sim[df_sim['Time'] <= cutoff]
        
        # Remove outliers
        for column in wheel_positions:
            df_sim = remove_outliers(df_sim, column)
        
        # Plot each wheel position's velocity data
        for i, column in enumerate(wheel_positions):
            color = colors[idx % len(colors)]  # Cycle through colors
            axs[i].plot(df_sim['Time'], df_sim[column], label=f'Sim {timestep_label} {column}', color=color, alpha=0.3)
            
            # Calculate and plot the average velocity line
            avg_sim = df_sim[column].mean()
            axs[i].axhline(avg_sim, color=color, linestyle=':', label=f'Sim {timestep_label} {column} Avg: {avg_sim:.4f}')

    # Calculate and plot real data averages
    for i, column in enumerate(wheel_positions):
        avg_real = df_real[column].mean()
        axs[i].plot(df_real['Time'], df_real[column], label=f'Real {column} (Raw)', color='red', alpha=0.5)
        axs[i].axhline(avg_real, color='red', linestyle='-', label=f'Real {column} Avg: {avg_real:.4f}')
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('Velocity (m/s)')
        axs[i].set_title(f'{column} Wheel Velocity (x={x_value}, y={y_value}, z={z_value}, secs={secs})')
        axs[i].grid(True)
        
        # Move legend outside of the plot
        axs[i].legend(loc='upper left', bbox_to_anchor=(1, 1))

    # Adjust layout to prevent overlap and to fit the legends
    plt.tight_layout(rect=[0, 0, 0.85, 1])

    # Show the plot
    plt.show()

def plot_wheel_velocities_compared(directory_simulation, directory_real_data, cutoff):
    # List all the subdirectories in the simulation directory
    steps_directories = [d for d in os.listdir(directory_simulation) if os.path.isdir(os.path.join(directory_simulation, d))]
    csv_directory_real_data = os.path.join(directory_real_data, "wheel_velocities")

    # Create a list of tuples (x_value, y_value, z_value, filename) to sort based on x_value, y_value, and z_value
    files_with_values = []

    # Loop through all CSV files in the real data directory
    for filename in os.listdir(csv_directory_real_data):
        if filename.endswith('.csv'):
            # Extract x, y, z, and secs values from the filename
            x_value = float(filename.split('x=')[1].split('y=')[0])
            y_value = float(filename.split('y=')[1].split('z=')[0])
            z_value = float(filename.split('z=')[1].split('secs=')[0])
            secs = filename.split('secs=')[1].split('.csv')[0]
            files_with_values.append((x_value, y_value, z_value, filename, secs))

    # Sort files based on the numeric x_value, y_value, and z_value
    files_with_values.sort(key=lambda x: (x[0], x[1], x[2]))

    # Process the sorted files
    for x_value, y_value, z_value, filename, secs in files_with_values:
        real_file = os.path.join(csv_directory_real_data, filename)

        # Collect corresponding simulation files from all timestep directories
        sim_files = []
        for timestep in steps_directories:
            sim_file = os.path.join(directory_simulation, timestep, "wheel_velocities", filename)
            if os.path.exists(sim_file):
                sim_files.append(sim_file)
        
        # Check if we found any simulation files for this real file
        if sim_files:
            # Pass the list of simulation files to the plotting function
            plot_wheel_velocities(sim_files, real_file, cutoff, x_value, y_value, z_value, secs)
        else:
            print(f"No corresponding simulation data found for real data file: {filename}")
